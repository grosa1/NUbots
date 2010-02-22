/*! @file Behaviour.cpp
    @brief Implementation of behaviour class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Behaviour.h"
#include <debug.h>
using namespace std;

Behaviour::Behaviour()
{
    
}


Behaviour::~Behaviour()
{
    
}

void Behaviour::process(JobList& jobs)
{

}
void Behaviour::processFieldObjects(FieldObjects* AllObjects,JobList& jobs)
{


    if(AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
    {
        vector<float> walkVector;
        walkVector.push_back(5);
        walkVector.push_back(0);
        walkVector.push_back(AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].Bearing()/2);
        WalkJob* walk = new WalkJob(walkVector);
        jobs.addMotionJob(walk);
        debug << "WalkJob created: Walk to BALL: "<< walkVector[0] << ","<<walkVector[1] <<"," << AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].Bearing()/2 << endl;
    }
}
