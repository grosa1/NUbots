#include "Tfidf.h"
#include <stdio.h>
#include <iostream>
#include <numeric>
#include "GoalMatcherConstants.h"
#include "RANSACLine.h"
#include "Ransac.h"


// Loads vocab ready for use
void Tfidf::loadVocab(std::string vocabFile) {

    // clear everything
    clearData();

    // load the vocab file
    vocab.loadVocabFile(vocabFile);
    T  = vocab.getSize();
    ni = Eigen::VectorXf::Zero(T);
    printf("Loaded vocab of %d words for GoalMatcher\n", T);
}

int Tfidf::getSize() {
    return N;
}

void Tfidf::clearMap() {
    clearData();
    T  = vocab.getSize();
    ni = Eigen::VectorXf::Zero(T);
}

float Tfidf::getValidCosineScore() {
    return VALID_COSINE_SCORE;
}

int Tfidf::getValidInliers() {
    return VALID_INLIERS;
}

void Tfidf::setValidCosineScore(float x) {
    VALID_COSINE_SCORE = x;
}

void Tfidf::setValidInliers(int x) {
    VALID_INLIERS = x;
}

// Loads map ready for use (needs a vocab first)

void Tfidf::loadMap(std::string mapFile) {

    if (T != 0) {
        // clear previous map
        clearMap();

        // load the map file
        std::vector<MapEntry> tempMap;
        std::ifstream ifs(mapFile.c_str());
        if (ifs.is_open()) {
            boost::archive::text_iarchive ia(ifs);
            ia >> tempMap;
        }
        else {
            throw std::runtime_error("error opening map file in tfidf");
        }

        for (int i = 0; i < (int) tempMap.size(); i++) {
            addDocumentToCorpus(tempMap.at(i));
        }
        printf("Loaded map with %d entries\n", map.size());
    }
}

// Saves map, including any new entries
void Tfidf::saveMap(std::string mapFile) {

    std::ofstream ofs(mapFile.c_str());
    {
        boost::archive::text_oarchive oa(ofs);
        oa << map;
    }
}

//! Adds to the searchable collection, return true if successful
bool Tfidf::addDocumentToCorpus(MapEntry document) {
    if (vocab.getSize() != 0) {
        std::unique_ptr<std::vector<std::vector<float>>> pixLoc = std::make_unique<std::vector<std::vector<float>>>();
        std::unique_ptr<std::vector<Ipoint>> ipoints_ptr        = std::make_unique<std::vector<Ipoint>>();
        *ipoints_ptr                                            = document.ipoints;
        Eigen::VectorXf tf_doc                                  = vocab.mapToVec(ipoints_ptr, pixLoc);
        return addDocumentToCorpus(document, tf_doc, *pixLoc);
    }
    return false;
}

//! Faster version if landmarks have already been mapped to words (with the same vocab file)
bool Tfidf::addDocumentToCorpus(MapEntry document, Eigen::VectorXf tf_doc, std::vector<std::vector<float>> pixLoc) {

    bool result = false;
    printf("addDocumentToCorpus:\n");
    if (vocab.getSize() != 0) {
        printf("vocab size criteria passed.\n");
        if (tf_doc.sum() != 0) {  // don't let an empty document be added
            printf("Adding tf_doc now (tf_doc sum = %f\n", tf_doc.sum());
            /*
            printf("tf_doc = [");
            int count = 0;
            for (int j = 0; j < tf_doc.size();j++){
                if (tf_doc[j] < 0.001){
                    count++;
                }
                else {
                    if (count > 0) printf("...//%d//...",count);
                    printf("(%.2f)",tf_doc[j]);
                    count = 0;
                }
            }
            if (count > 0) printf("...//%d//...",count);
            printf("]\n\n");
            */
            tf.push_back(tf_doc);
            pixels.push_back(pixLoc);
            ni = ni + tf_doc;
            N++;
            nd.push_back(tf_doc.sum());
            map.push_back(document);
            //! Recalculate the inverse document frequency (log(N/ni))
            idf = (ni.array().inverse() * N).log();
            // Remove Inf from idf (can occur with random initialised learned words)
            for (int i = 0; i < T; i++) {
                if (idf[i] == std::numeric_limits<float>::infinity()) {
                    idf[i] = 0.f;
                }
            }
            /*
            printf("idf = [");
            count = 0;
            for (int j = 0; j < idf.size();j++){
                if (idf[j] > 88.0){
                    count++;
                }
                else {
                    if (count > 0) printf("...//%d//...",count);
                    printf("(%.2f)",idf[j]);
                    count = 0;
                }
            }
            if (count > 0) printf("...//%d//...",count);
            printf("]\n\n");
            */
            result = true;
        }
        else {
            printf("tf_doc was an empty document\n");
        }
    }
    else {
        printf("vocab size criteria FAILED\n");
    }
    return result;
}

//! Faster version if landmarks have already been mapped to words
void Tfidf::searchDocument(Eigen::VectorXf tf_query,
                           std::vector<std::vector<float>> query_pixLoc,  // pixel locations of the words
                           std::unique_ptr<std::priority_queue<MapEntry>>& matches,
                           unsigned int* seed,
                           int num,
                           Eigen::MatrixXd* resultTable) {

    NUClear::clock::time_point t = NUClear::clock::now();
    printf("tf_query.sum = %.1f, N = %d\n", tf_query.sum(), N);
    if (tf_query.sum() != 0 && N != 0) {  // checked the document is not empty and corpus not empty
        printf("Running Tfidf::searchDocument now.\n");
        std::priority_queue<std::pair<MapEntry, std::vector<std::vector<float>>>> queue;
        Eigen::VectorXf tfidf_query = (tf_query / tf_query.sum()).array() * idf.array();

        int count = 0;
        /*
        printf("tf_query =    [");
        for (int j = 0; j < tf_query.size();j++){
            if (tf_query[j] < 0.001){
                count++;
            }
            else {
                if (count > 0) printf("...//%d//...",count);
                printf("(%.2f)",tf_query[j]);
                count = 0;
            }
        }
        if (count > 0) printf("...//%d//...",count);
        printf("]\n\n");
        */
        /*
        printf("tfidf_query = [");
        count = 0;
        for (int j = 0; j < tfidf_query.size();j++){
            if (tfidf_query[j] < 0.001){
                count++;
            }
            else {
                if (count > 0) printf("...//%d//...",count);
                printf("(%.2f)",tfidf_query[j]);
                count = 0;
            }
        }
        if (count > 0) printf("...//%d//...",count);
        printf("]\n\n");
        */

        // Now compute the cosines against each document -- inverted index not used since our vectors are not sparse
        // enough
        Eigen::VectorXf tfidf_doc;
        printf("Cosine scores: (for N=%d)\n", N);
        for (int i = 0; i < N; i++) {
            tfidf_doc = (tf[i] / nd[i]).array() * idf.array();  // nd[i] can't be zero or it wouldn't have been added

            // printf("nd[i] = %d\n",nd[i]);
            /*
            printf("tfidf_doc = [");
            int count = 0;
            for (int j = 0; j < tfidf_doc.size();j++){
                if (tfidf_doc[j] < 0.001){
                    count++;
                }
                else {
                    if (count > 0) printf("...//%d//...",count);
                    printf("(%.2f)",tfidf_doc[j]);
                    count = 0;
                }
            }
            if (count > 0) printf("...//%d//...",count);
            printf("]\n");
            */
            // printf("tfidf_query dot tfidf_doc : %f, tfidf_query norm: %.2f, tfidf_doc norm:
            // %.2f\n",tfidf_query.dot(tfidf_doc),tfidf_query.norm(),tfidf_doc.norm());

            map.at(i).score = cosineScore(tfidf_query, tfidf_doc);
            printf("%2d. map score: %.2f   ", i + 1, map[i].score);
            if (map.at(i).score > VALID_COSINE_SCORE) {
                printf("This is a valid cosine score");
                queue.push(std::make_pair(map.at(i), pixels.at(i)));
                /*
                printf("tfidf_doc = [");
                int count = 0;
                for (int j = 0; j < tfidf_doc.size();j++){
                    if (tfidf_doc[j] < 0.001){
                        count++;
                    }
                    else {
                        if (count > 0) printf("...//%d//...",count);
                        printf("(%.2f)",tfidf_doc[j]);
                        count = 0;
                    }
                }
                if (count > 0) printf("...//%d//...",count);
                printf("]\n");
                */
            }
            printf("\n");
        }
        printf("Complete.\n");

        // Now do geometric validation on the best until we have enough or the queue is empty
        int counter = -1;
        while (!queue.empty() && matches->size() < (unsigned int) num) {
            MapEntry mapEntry = queue.top().first;
            printf("Validating Cos: %.2f ", mapEntry.score);
            counter++;
            (*resultTable)(counter, 0) = mapEntry.score;
            if ((mapEntry.position.theta() < M_PI / 2) && (mapEntry.position.theta() > -M_PI / 2)) {
                printf("(OPP)");
                (*resultTable)(counter, 1) = 1.0;
                (*resultTable)(0, 4)       = (*resultTable)(0, 4) + 1.0;
            }
            else {
                printf("(OWN)");
                (*resultTable)(counter, 1) = -1.0;
                (*resultTable)(0, 5)       = (*resultTable)(0, 5) + 1.0;
            }
            std::vector<std::vector<float>> pixLoc = queue.top().second;
            queue.pop();

            // Precomputing match info
            std::vector<Eigen::VectorXf> match_tfidf_subL432 = spatialPyramidMatchPreCalc(pixLoc);

            // Spatial Pyramid geometric validation
            auto SPtimer_s            = std::chrono::system_clock::now();
            float spatialPyramidScore = spatialPyramidCheck(match_tfidf_subL432, query_pixLoc);
            auto SPtime_e             = std::chrono::system_clock::now();
            auto SPtimer              = std::chrono::duration_cast<std::chrono::microseconds>(SPtime_e - SPtimer_s);
            addToSPAverage((float) SPtimer.count());
            std::cout << ", SPtimer: " << SPtimer.count() << " us";

            auto RANSACtimer_s = std::chrono::system_clock::now();
            // Do geometric validation - first build the points to run ransac
            std::vector<Point> matchpoints;
            for (int j = 0; j < T; j++) {
                for (uint32_t m = 0; m < pixLoc[j].size(); m++) {
                    for (uint32_t n = 0; n < query_pixLoc[j].size(); n++) {
                        matchpoints.push_back(Point(pixLoc[j][m], query_pixLoc[j][n]));
                        // printf("matchpoints: %.1f, %.1f\n",pixLoc[j][m], query_pixLoc[j][n]);
                    }
                }
            }

            // ransac
            RANSACLine resultLine;
            uint16_t min_points = 2;
            std::vector<bool>*con, consBuf[2];
            consBuf[0].resize(matchpoints.size());
            consBuf[1].resize(matchpoints.size());
            float slopeConstraint = 2.f;  // reject scale changes more than twice or half the distance
            bool ransacresult     = RANSAC::findLineConstrained(matchpoints,
                                                            &con,
                                                            resultLine,
                                                            MATCH_ITERATIONS,
                                                            PIXEL_ERROR_MARGIN,
                                                            min_points,
                                                            consBuf,
                                                            seed,
                                                            slopeConstraint);

            auto RANSACtimer_e = std::chrono::system_clock::now();
            auto RANSACtimer   = std::chrono::duration_cast<std::chrono::microseconds>(RANSACtimer_e - RANSACtimer_s);
            addToRANSACAverage((float) RANSACtimer.count());
            std::cout << ", RANSACtimer: " << RANSACtimer.count() << " us";

            if (ransacresult && (resultLine.t2 != 0.f)) {  // check t2 but should be fixed by slope constraint anyway
                // count the inliers
                int inliers = 0;
                for (int v = 0; v < (int) matchpoints.size(); v++) {
                    if ((*con)[v]) {
                        inliers++;
                    }
                }
                printf(", found %d inliers, %d outliers, ", inliers, (int) matchpoints.size() - inliers);
                (*resultTable)(counter, 2) = (double) inliers;
                (*resultTable)(counter, 3) = (double) matchpoints.size() - (double) inliers;
                // printf("at (x,y,theta) loc: (%.1f, %.1f, %.1f)\n", mapEntry.position.x(), mapEntry.position.y(),
                // mapEntry.position.theta());

                if (inliers >= VALID_INLIERS) {
                    printf("Location is valid\n");
                    matches->push(mapEntry);
                }
                else {
                    printf("Location INVALID\n");
                    num--;
                    if ((*resultTable)(counter, 1) > 0.5) (*resultTable)(0, 4)  = (*resultTable)(0, 4) - 1.0;
                    if ((*resultTable)(counter, 1) < -0.5) (*resultTable)(0, 5) = (*resultTable)(0, 5) - 1.0;
                }
            }
            else {
                printf("SKIPPED INLIER COUNT SINCE ransacresult = %d\n", ransacresult);
                num--;
                if ((*resultTable)(counter, 1) > 0.5) (*resultTable)(0, 4)  = (*resultTable)(0, 4) - 1.0;
                if ((*resultTable)(counter, 1) < -0.5) (*resultTable)(0, 5) = (*resultTable)(0, 5) - 1.0;
            }
        }
    }
    // printf("\nCalculating cosines over " << map.size() << " images, RANSAC geo validation & position adjustments took
    // ";
    // llog(DEBUG1) << t.elapsed_us() << " us" << std::endl;
}

float Tfidf::cosineScore(Eigen::VectorXf a, Eigen::VectorXf b) {
    return a.dot(b) / (std::max(double(a.norm() * b.norm()), 0.0000000000001));
}

float Tfidf::PearsonsCorrelation(Eigen::VectorXf a, Eigen::VectorXf b) {
    float sumAB  = (a.array() * b.array()).sum();
    float sumA   = a.array().sum();
    float sumB   = b.array().sum();
    float sumAsq = a.array().square().sum();
    float sumBsq = b.array().square().sum();
    float n      = a.size();

    return (sumAB - sumA * sumB / n) / (sqrt((sumAsq - sumA * sumA / n) * (sumBsq - sumB * sumB / n)));
}

/*
 * L3:     |  2  |  5  |  8  |  11 |      <- 3 block groupings, offset by 2
 * L3:   |  1  |  4  |  7  |  10 |  13 |  <- 3 block groupings, offset by 1
 * L3: |  0  |  3  |  6  |  9  |  12 |    <- 3 block groupings
 * L1: | | | | | | | | | | | | | | | | |
 */

float Tfidf::spatialPyramidCheck(std::vector<Eigen::VectorXf> match_tfidf_subL432,
                                 std::vector<std::vector<float>> query_pixLoc) {


    int xmin     = 0;
    int stepsize = 20;

    auto timer1s = std::chrono::system_clock::now();
    // each element of outside vector contains the tf for a pixel subset
    std::vector<Eigen::VectorXf> query_tf_subL1;
    // Splitting up the x positions into blocks the size of 'stepsize'.
    while ((xmin + stepsize) <= IMAGE_WIDTH) {
        query_tf_subL1.push_back(Eigen::VectorXf::Zero(query_pixLoc.size()));

        for (int i = 0; i < query_pixLoc.size(); ++i) {  // steps through each feature
            for (int j = 0; j < query_pixLoc[i].size(); ++j) {
                if ((query_pixLoc[i][j] >= (float) xmin) && (query_pixLoc[i][j] < (float) (xmin + stepsize))) {
                    query_tf_subL1.back()[i] += 1.0;
                }
            }
        }
        xmin += stepsize;
    }
    auto timer1e = std::chrono::system_clock::now();
    auto timer1  = std::chrono::duration_cast<std::chrono::microseconds>(timer1e - timer1s);
    std::cout << ", t1: " << timer1.count() << "us";

    // Combining the base of the pyramid blocks to create the layer 3
    auto timer2s = std::chrono::system_clock::now();
    std::vector<Eigen::VectorXf> query_tfidf_subL3;
    Eigen::VectorXf tf_sub_temp;
    int i_subset = 0;
    xmin         = 0;  // resetting
    int L_max    = 3;
    int L_min    = 3;
    for (int L = L_max; L >= L_min; --L) {
        while ((xmin + stepsize * L) <= IMAGE_WIDTH) {
            tf_sub_temp = query_tf_subL1[i_subset];
            for (int i = 1; i < L; ++i) {
                tf_sub_temp += query_tf_subL1[i_subset + i];
            }
            query_tfidf_subL3.push_back((tf_sub_temp / tf_sub_temp.sum()).array() * idf.array());
            xmin += stepsize;
            i_subset++;
        }
        xmin     = 0;
        i_subset = 0;
    }
    auto timer2e = std::chrono::system_clock::now();
    auto timer2  = std::chrono::duration_cast<std::chrono::microseconds>(timer2e - timer2s);
    std::cout << ", t2: " << timer2.count() << "us";

    // Comparing the blocks on level 3
    auto timer3s     = std::chrono::system_clock::now();
    int L            = 3;
    float maxSPScore = 0;
    float SPScore    = 0;
    int i_match      = 19;  // The sliding image
    int i_query      = 0;   // The stationary image
    int n_blocks     = 3;

    for (int i = 1; i <= 12; ++i) {
        if (i == 3) {
            n_blocks = 4;
        }
        else if (i == 6) {
            n_blocks = 5;
        }
        else if (i == 8) {
            n_blocks = 4;
            i_match  = 15;  // The sliding image
            i_query  = 3;   // The stationary image
        }
        else if (i == 11) {
            n_blocks = 3;
            i_match  = 15;  // The sliding image
            i_query  = 6;   // The stationary image
        }

        for (int j = 0; j < n_blocks; ++j) {
            SPScore += cosineScore(query_tfidf_subL3[i_query + j * L], match_tfidf_subL432[i_query + j * L]);
        }
        SPScore /= n_blocks;  // Average
        if (SPScore > maxSPScore) {
            maxSPScore = SPScore;
        }
        SPScore = 0;
        i_match--;
    }
    auto timer3e = std::chrono::system_clock::now();
    auto timer3  = std::chrono::duration_cast<std::chrono::microseconds>(timer3e - timer3s);
    std::cout << ", t3: " << timer3.count() << "us";

    // Comparing the level 3 blocks from query, with the level 4 blocks from match
    auto timer4s = std::chrono::system_clock::now();
    i_match      = 0;
    i_query      = 7;
    n_blocks     = 3;
    for (int i = 1; i <= 11; ++i) {
        if (i == 4) {
            n_blocks = 4;
        }
        else if (i == 9) {
            n_blocks = 3;
            i_query  = 2;  // The sliding image
            i_match  = 4;  // The stationary image
        }

        for (int j = 0; j < n_blocks; ++j) {
            SPScore += cosineScore(query_tfidf_subL3[i_query + j * L], match_tfidf_subL432[i_match + j * (L + 1)]);
        }
        SPScore /= n_blocks;  // Average
        if (SPScore > maxSPScore) {
            maxSPScore = SPScore;
        }
        SPScore = 0;
        i_query--;
    }
    auto timer4e = std::chrono::system_clock::now();
    auto timer4  = std::chrono::duration_cast<std::chrono::microseconds>(timer4e - timer4s);
    std::cout << ", t4: " << timer4.count() << "us";

    // Comaring the level 3 blocks from query, with the level 2 blocks from match
    auto timer5s = std::chrono::system_clock::now();
    i_match      = 34;  // The sliding image
    i_query      = 0;   // The stationary image
    n_blocks     = 4;
    for (int i = 1; i <= 9; ++i) {
        if (i == 2) {
            n_blocks = 5;
        }
        else if (i == 9) {
            n_blocks = 4;
            i_match  = 28;
            i_query  = 3;
        }

        for (int j = 0; j < n_blocks; ++j) {
            SPScore += cosineScore(query_tfidf_subL3[i_query + j * L], match_tfidf_subL432[i_match + j * (L - 1)]);
        }
        SPScore /= n_blocks;  // Average
        if (SPScore > maxSPScore) {
            maxSPScore = SPScore;
        }
        SPScore = 0;
        i_match--;
    }
    auto timer5e = std::chrono::system_clock::now();
    auto timer5  = std::chrono::duration_cast<std::chrono::microseconds>(timer5e - timer5s);
    std::cout << ", t5: " << timer5.count() << "us";
    printf(" (maxSPScore: %0.2f)", maxSPScore);

    return maxSPScore;
}


/*
 * L4:       |   3   |   7   |   11  |    <- 4 block groupings, offset by 3
 * L4:     |   2   |   6   |   10  |      <- 4 block groupings, offset by 2
 * L4:   |   1   |   5   |   9   |        <- 4 block groupings, offset by 1
 * L4: |   0   |   4   |   8   |   12  |  <- 4 block groupings
 * L3:     |  15 |  18 |  21 |  24 |      <- 3 block groupings, offset by 2
 * L3:   |  14 |  17 |  20 |  23 |  26 |  <- 3 block groupings, offset by 1
 * L3: |  13 |  16 |  19 |  22 |  25 |    <- 3 block groupings
 * L2:   | 28| 30| 32| 34| 36| 38| 40|    <- 2 block groupings, offset by 1
 * L2: | 27| 29| 31| 33| 35| 37| 39| 41|  <- 2 block groupings
 * L1: | | | | | | | | | | | | | | | | |
 */
std::vector<Eigen::VectorXf> Tfidf::spatialPyramidMatchPreCalc(std::vector<std::vector<float>> match_pixLoc) {
    // each element of outside vector contains the tf for a pixel subset
    std::vector<Eigen::VectorXf> match_tf_subL1;
    int xmin     = 0;
    int stepsize = 20;

    // Splitting up the x positions into blocks the size of 'stepsize'.
    while ((xmin + stepsize) <= IMAGE_WIDTH) {
        match_tf_subL1.push_back(Eigen::VectorXf::Zero(match_pixLoc.size()));

        for (int i = 0; i < match_pixLoc.size(); ++i) {  // steps through each feature
            for (int j = 0; j < match_pixLoc[i].size(); ++j) {
                // store all features with an x position within each block
                if ((match_pixLoc[i][j] >= (float) xmin) && (match_pixLoc[i][j] < (float) (xmin + stepsize))) {
                    match_tf_subL1.back()[i] += 1.0;
                }
            }
        }
        xmin += stepsize;
    }

    // Combining the base of the pyramid blocks to create the high layers (4, 3 then 2)
    std::vector<Eigen::VectorXf> match_tfidf_subL432;
    Eigen::VectorXf tf_sub_temp;
    int i_subset = 0;
    xmin         = 0;  // resetting
    int L_max    = 4;
    int L_min    = 2;
    for (int L = L_max; L >= L_min; --L) {
        while ((xmin + stepsize * L) <= IMAGE_WIDTH) {
            tf_sub_temp = match_tf_subL1[i_subset];
            for (int i = 1; i < L; ++i) {
                tf_sub_temp += match_tf_subL1[i_subset + i];
            }
            match_tfidf_subL432.push_back((tf_sub_temp / tf_sub_temp.sum()).array() * idf.array());
            xmin += stepsize;
            i_subset++;
        }
        xmin     = 0;
        i_subset = 0;
    }
    if (match_tfidf_subL432.size() != 42) {
        std::cout << "ERROR!!! match_tfidf_subL432 is not expected size" << std::endl;
    }
    return match_tfidf_subL432;
}

void Tfidf::addToSPAverage(float time) {
    SPAverageN += 1.0f;
    if (SPAverageN < 1.5f) {
        SPAverageTime = time;
    }
    else {
        SPAverageTime = ((SPAverageN - 1.0f) / SPAverageN) * SPAverageTime + time / SPAverageN;
    }
}

void Tfidf::addToRANSACAverage(float time) {
    RANSACAverageN += 1.0f;
    if (RANSACAverageN < 1.5f) {
        RANSACAverageTime = time;
    }
    else {
        RANSACAverageTime = ((RANSACAverageN - 1.0f) / RANSACAverageN) * RANSACAverageTime + time / RANSACAverageN;
    }
}

void Tfidf::printRANSACandSPAverages() {
    float ratio = SPAverageTime / RANSACAverageTime;
    printf("SPAverageTime: %0.0f, RANSACAverageTime: %0.0f (x%0.1f)\n", SPAverageTime, RANSACAverageTime, ratio);
}