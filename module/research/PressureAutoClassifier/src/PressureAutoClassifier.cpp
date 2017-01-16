/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "PressureAutoClassifier.h"

#include "message/research/AutoClassifierPixels.h"
#include "message/support/Configuration.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/proto/LookUpTableDiff.h"

namespace module {
namespace research {

    using message::input::Image;
    using message::research::AutoClassifierPixels;
    using message::support::Configuration;
    using message::vision::LookUpTable;
    using message::vision::proto::LookUpTableDiff;
    using message::vision::Colour;


    /**
     * @brief Gets the index int the LUT given an x, y and z coordinate for it
     *
     * @param lut the lookup table that we want an index for
     * @param x the x coordinate we are using
     * @param y the y coordinate we are using
     * @param z the z coordinate we are using
     *
     * @return the array index that represents these coordinates in the lut
     */
    inline uint getIndex(const LookUpTable& lut, const uint8_t& x, const uint8_t& y, const uint8_t& z) {
        return (((x << lut.BITS_CR) | y) << lut.BITS_CB) | z;
    }

    /**
     * @brief Given a lut and an index to an array element in the lut, Exctracts the x, y and z coordinates for this index
     *
     * @param lut the lookup table that we want an index for
     * @param index the array index that represents these coordinates in the lut
     *
     * @return the x, y and z coordinates that this index represents
     */
    inline std::array<uint, 3> getCoordinates(const LookUpTable& lut, const uint& index) {

        uint x = (index >> (lut.BITS_CB + lut.BITS_CR)) & ((1 << lut.BITS_Y) - 1);
        uint y = (index >> lut.BITS_CR) & ((1 << lut.BITS_CB) - 1);
        uint z = index & ((1 << lut.BITS_CR) - 1);

        return { x, y, z };
    }

    /**
     * @brief Gets the Colour that is at this position in the lookup table
     *
     * @param lut the lookup table that we are searching in
     * @param index the array index that represents these coordinates in the lut
     *
     * @return the colour that is located at this index in the lut
     */
    inline const Colour& getAt(const LookUpTable& lut, const uint& index) {
        return lut.getRawData()[index];
    }

    /**
     * @brief Gets the Colour that is at this position in the lookup table
     *
     * @param lut the lookup table that we are searching in
     * @param index the array index that represents these coordinates in the lut
     *
     * @return the colour that is located at this index in the lut
     */
    inline Colour& getAt(LookUpTable& lut, const uint& index) {
        return lut.getRawData()[index];
    }

    /**
     * @brief Checks if the passed index is touching the passed colour on any of its faces.
     *
     * @details A voxel is considered touching the colour if any of its faces touch a cell
     *      that is that colour. This makes for 6 possible cells to check against.
     *
     * @param lut the lookup table that we are searching in
     * @param index the array index that represents these coordinates in the lut
     * @param c the colour that we are looking for
     *
     * @return true if the index is touching the colour c, false otherwise
     */
    inline bool isTouching(const LookUpTable& lut, const uint& index, const Colour& c) {

        // Loop through each axis and check any are filled
        for(uint i : { 1, 1 << lut.BITS_CR, 1 << (lut.BITS_CB + lut.BITS_CR) }) {

            // If either direction is the colour we are looking for we are touching
            if(getAt(lut, index + i) == c
            || getAt(lut, index - i) == c) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Checks if the passed index is an internal voxel
     *
     * @details A voxel is considered internal if all of it's faces are touching voxels
     *          that are the same colour as itself
     *
     * @param lut the lookup table that we are searching in
     * @param index the array index that represents these coordinates in the lut
     *
     * @return true if the index is internal, false otherwise
     */
    inline bool isInternal(const LookUpTable& lut, const uint& index) {

        // Get the classification for this voxel
        Colour c = getAt(lut, index);

        // Will be true if this voxel is internal
        bool internal = true;

        // Loop through each axis and check they are all filled
        for(uint i : { 1, 1 << lut.BITS_CR, 1 << (lut.BITS_CB + lut.BITS_CR) }) {
            internal &= getAt(lut, index + i) == c;
            internal &= getAt(lut, index - i) == c;
        }

        return internal;
    }

    /**
     * @brief Returns the list of voxels which the passed voxel could influence
     *
     * @details This function will get all of the voxels that the passed index could influence.
     *          This includes all of the voxels it is touching plus the sideways diagonals.
     *
     * @param lut lut the lookup table that we are searching in
     * @param index the array index that represents these coordinates in the lut
     *
     * @return a list of all the voxels that this index could influence
     */
    inline std::set<uint> influence(const LookUpTable& lut, const uint& index) {

        std::set<uint> inf;

        // Each of our touchign faces
        for(uint i : { 1, 1 << lut.BITS_CR, 1 << (lut.BITS_CB + lut.BITS_CR) }) {
            inf.insert(index + i);
            inf.insert(index - i);
        }

        // Insert our side diagonals
        for(uint i : {1 << lut.BITS_CR, 1 << (lut.BITS_CB + lut.BITS_CR) }) {

            inf.insert(index + i + 1);
            inf.insert(index - i + 1);
            inf.insert(index + i - 1);
            inf.insert(index - i - 1);
        }

        return inf;
    }

    /**
     * @brief Checks if the passed index is removeable from the LUT as a surface removeable voxel.
     *
     * @details This will check if the voxel at index can be safely removed from the LUT as the surface.
     *          This will be true if any of the following are true:
     *            1. the voxel does not have two opposite classified voxels.
     *            2. the voxel has two non oppisite internal voxels (internal voxels are voxels with all faces filled)
     *            3. the voxel has at least 3 voxels
     *
     *
     * @param lut [descr iption]
     * @param index [description]
     *
     * @return
     */
    inline bool isRemoveable(const LookUpTable& lut, const uint& index) {

        // Get the classification for this pixel
        Colour c = getAt(lut, index);

        // Internal is never removeable
        if(isInternal(lut, index)) {
            return false;
        }

        // Will be true if this voxel is removeable
        bool removeable = false;

        // Loop through each axis
        for(uint i : { 1, 1 << lut.BITS_CR, 1 << (lut.BITS_CB + lut.BITS_CR) }) {

            // Check if we have a filled cell opposite to a non filled cell
            if(!(getAt(lut, index + i) == c && getAt(lut, index - i) == c)) {
                removeable = true;
            }
            else {
                // Break! this isn't surface by this criteria!
                removeable = false;
                break;
            }
        }

        // If we are not removeable yet
        if(!removeable) {

            // The number of internal voxels
            uint internal = 0;
            // The number of voxels that are internal and not opposite another internal
            uint nonOppositeInternal = 0;

            // Loop through each axis again
            for(uint i : { 1, 1 << lut.BITS_CR, 1 << (lut.BITS_CB + lut.BITS_CR) }) {

                // Check if either side is internal
                bool a = getAt(lut, index + i) == c && isInternal(lut, index + 1);
                bool b = getAt(lut, index - i) == c && isInternal(lut, index - 1);

                // Add to our internal counts
                internal += a + b;
                nonOppositeInternal += a != b;

                if(internal >= 3 || nonOppositeInternal >= 2) {
                    // We are now removeable
                    removeable = true;
                    break;
                }
            }
        }

        return removeable;
    }

    /**
     * @brief Shed the outer layer of the lookup table
     *
     * @details Takes all of the voxels that make up the "surface" of the LUT (as defined by isRemoveable)
     *          and removes them from the LUT.
     *
     * @param lut our lut that we are shedding layers from
     * @param c the colour that we are shedding for
     * @param sa the set of surface area removeable voxels we are removing (and refilling with the new surface)
     * @param vol The volume of this colour (to be modified when removing)
     */
    void shed(LookUpTable& lut, Colour c, std::set<uint>& sa, uint& vol, std::map<uint, uint>& votes) {
        // Holds our new surface voxels
        std::set<uint> newSA;
        std::set<uint> heldSA;

        // Get the mean of the surface
        uint n = 0, v = 0;
        for(auto& s : sa) {
            ++n;
            v += votes[s];
        }
        uint mean = v/n;

        // Move all SA elements above the mean to the new list and remove them from the old list
        // Then we can remove all elements from the old list from the surface
        for(auto it = sa.begin(); it != sa.end();) {
            if(votes[*it] >= mean) {
                heldSA.insert(*it);
                it = sa.erase(it);
            }
            // Otherwise move on
            else {
                ++it;
            }
        }

        // Go through the surface area for this colour
        for(auto& s : sa) {

            // Everything we were influencing is potential SA
            std::set<uint> inf = influence(lut, s);
            newSA.insert(std::begin(inf), std::end(inf));

            // Set this voxel to unclassified and remove it from the volume
            getAt(lut, s) = Colour::UNCLASSIFIED;
            --vol;

            // Remove from our vote map
            votes.erase(votes.find(s));
        }

        // Remove all non SA points from our new SA
        for(auto it = std::begin(newSA); it != std::end(newSA);) {
            auto& p = *it;

            // Remove if it's not the correct colour (it was also an SA voxel and was removed)
            // Remove if it's not an SA voxel now
            if(getAt(lut, p) != c || !isRemoveable(lut, p)) {
                it = newSA.erase(it);
            }
            // Otherwise move on
            else {
                ++it;
            }
        }

        // Insert our held SA
        newSA.insert(heldSA.begin(), heldSA.end());

        // Store our new surface area
        sa = std::move(newSA);
    }

    PressureAutoClassifier::PressureAutoClassifier(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("PressureAutoClassifier.yaml").then([this](const Configuration& config) {

            //Loop through each classification char
            for(auto& limit : config["limits"]) {

                Colour c          = static_cast<Colour>(limit.first.as<char>());
                maxVolume[c]      = limit.second["max_volume"].as<uint>();
                maxSurfaceArea[c] = limit.second["surface_area_volume_ratio"].as<double>() * maxVolume[c];
                zeroPoints[c]       = 0;
                zeroPointGrowths[c] = limit.second["zero_point_growth"].as<uint>();
                voteGrowths[c]      = limit.second["vote_growth"].as<uint>();
                maxVotes[c]         = limit.second["max_votes"].as<uint>();
            }
        });

        // When we get a look up table then it was uploaded or emitted by someone else
        // We need to set it up for our datastructure
        on<Trigger<LookUpTable>>().then([this] (const LookUpTable& lut) {

            std::map<Colour, std::set<uint>> newSA;
            std::map<Colour, uint> newVol;
            std::map<uint, uint> newVotes;

            // Loop through every voxel in the lut
            for(uint x = 0; x < uint(1 << lut.BITS_Y); ++x) {
                for (uint y = 0; y < uint(1 << lut.BITS_CB); ++y) {
                    for (uint z = 0; z < uint(1 << lut.BITS_CR); ++z) {

                        // Get our index and classification
                        uint index = getIndex(lut, x, y, z);
                        Colour c = getAt(lut, index);

                        // Ignore unclassified pixels
                        if(c != Colour::UNCLASSIFIED) {

                            // Increase our volume
                            ++newVol[c];

                            // Set with our maximum number of votes
                            newVotes.emplace(index, maxVotes[c]);

                            // If this removeable surface then add it to our list
                            if(isRemoveable(lut, index)) {
                                newSA[c].insert(index);
                            }
                        }
                    }
                }
            }

            surfaceArea = std::move(newSA);
            volume = std::move(newVol);
            votes = std::move(newVotes);
        });

        // Show our Internal, Removeable, and Nonremoveable voxels
        on<Every<1, std::chrono::seconds>, With<LookUpTable>>().then([this] (const LookUpTable& lut) {
            static int i = 0;

            // For visualization we are flagging surface voxels
            auto tableDiff = std::make_unique<LookUpTableDiff>();

            // Show the removeable surface
            if(i % 3 == 0) {
                for(uint x = 0; x < uint(1 << lut.BITS_Y); ++x) {
                    for (uint y = 0; y < uint(1 << lut.BITS_CB); ++y) {
                        for (uint z = 0; z < uint(1 << lut.BITS_CR); ++z) {

                            // Get our relevant information
                            uint index = getIndex(lut, x, y, z);
                            Colour c = getAt(lut, index);

                            // Now time to choose the colour
                            if(c == Colour::YELLOW && isRemoveable(lut, index)) {
                                auto& diff = *tableDiff->add_diff();
                                diff.set_lut_index(index);
                                diff.set_classification(Colour::CYAN);
                            }
                        }
                    }
                }
            }

            // Show the internal voxels
            else if(i % 3 == 1) {

                for(uint x = 0; x < uint(1 << lut.BITS_Y); ++x) {
                    for (uint y = 0; y < uint(1 << lut.BITS_CB); ++y) {
                        for (uint z = 0; z < uint(1 << lut.BITS_CR); ++z) {

                            // Get our relevant information
                            uint index = getIndex(lut, x, y, z);
                            Colour c = getAt(lut, index);

                            // Now time to choose the colour
                            if(c == Colour::YELLOW && isInternal(lut, index)) {
                                auto& diff = *tableDiff->add_diff();
                                diff.set_lut_index(index);
                                diff.set_classification(Colour::YELLOW);
                            }
                        }
                    }
                }

            }

            // Show the other voxels
            if(i % 3 == 2) {

                for(uint x = 0; x < uint(1 << lut.BITS_Y); ++x) {
                    for (uint y = 0; y < uint(1 << lut.BITS_CB); ++y) {
                        for (uint z = 0; z < uint(1 << lut.BITS_CR); ++z) {

                            // Get our relevant information
                            uint index = getIndex(lut, x, y, z);
                            Colour c = getAt(lut, index);

                            // Now time to choose the colour
                            if(c == Colour::YELLOW && !isRemoveable(lut, index) && !isInternal(lut, index)) {
                                auto& diff = *tableDiff->add_diff();
                                diff.set_lut_index(index);
                                diff.set_classification(Colour::MAGENTA);
                            }
                        }
                    }
                }

            }

            emit(std::move(tableDiff));

            // Next time show the next set
            ++i;
        }).disable();

        on<Trigger<AutoClassifierPixels>, With<LookUpTable>, Single>()
        .then([this] (const AutoClassifierPixels& pixels, const LookUpTable& lut) {

            // Some aliases
            const auto& c         = pixels.classification;
            auto& vol             = volume[c];
            auto& maxVol          = maxVolume[c];
            auto& sa              = surfaceArea[c];
            auto& maxSA           = maxSurfaceArea[c];
            auto& zeroPoint       = zeroPoints[c];
            auto& zeroPointGrowth = zeroPointGrowths[c];
            auto& maxVote         = maxVotes[c];
            auto& voteGrowth      = voteGrowths[c];

            // TODO increase the 0 point for this voxel
            zeroPoint += zeroPointGrowth;

            // Build up our differences in the look up table
            auto tableDiff = std::make_unique<LookUpTableDiff>();

            for(auto& p : pixels.pixels) {

                // Lookup the pixel
                auto colour = lut(p);

                // Get our voxel coordinates for this pixel
                uint index = lut.getLUTIndex(p);

                // If it's an unclassified pixel then we can do something
                if(colour == Colour::UNCLASSIFIED) {

                    // Check if we are touching a filled voxel
                    if(isTouching(lut, index, c)) {

                        // TODO Initialise our votes for this cell
                        votes.emplace(index, zeroPoint);

                        // We are going to need a mutable lut
                        LookUpTable& mLut = *const_cast<LookUpTable*>(&lut);

                        // If we have exceeded max volume then shed
                        if(vol >= maxVol) {
                            // Emit the diff of the voxels we are about to remove
                            for(auto& s : sa) {
                                // Add our diff for displaying
                                auto& diff = *tableDiff->add_diff();
                                diff.set_lut_index(s);
                                diff.set_classification(Colour::UNCLASSIFIED);
                            }

                            // Shed our voxel layer
                            shed(mLut, c, sa, vol, votes);
                        }
                        // Otherwise we can classify this
                        else {
                            // Classify
                            getAt(mLut, index) = c;

                            // Volume increase
                            ++vol;

                            // If the new voxel is SA add it to the SA list
                            if(isRemoveable(lut, index)) {
                                sa.insert(index);
                            }

                            // Loop through our influenced voxels and update their status
                            for(uint i : influence(lut, index)) {

                                // We only care about our own colour
                                if(getAt(lut, i) == c) {
                                    if(isRemoveable(lut, i)) {
                                        sa.insert(i);
                                    }
                                    else if(sa.find(i) != std::end(sa)) {
                                        sa.erase(sa.find(i));
                                    }
                                }
                            }

                            // Add our diff for displaying
                            auto& diff = *tableDiff->add_diff();
                            diff.set_lut_index(index);
                            diff.set_classification(c);

                            // If we exceeded our SA constraint then shed
                            if(sa.size() >= maxSA) {
                                // Emit the diff of the voxels we are about to remove
                                for(auto& s : sa) {
                                    // Add our diff for displaying
                                    auto& diff = *tableDiff->add_diff();
                                    diff.set_lut_index(s);
                                    diff.set_classification(Colour::UNCLASSIFIED);
                                }

                                shed(mLut, c, sa, vol, votes);
                            }
                        }
                    }
                }
                else {

                    // TODO increase the votes for this cell
                    votes[index] = std::min(uint(zeroPoint + maxVote), uint(votes[index] + voteGrowth));
                }
            }

            if (tableDiff->diff_size() > 0) {
                emit(std::move(tableDiff));
            }
        });
    }

}
}

