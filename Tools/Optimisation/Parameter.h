/*! @file Parameter.h
    @brief Declaration of Parameter class
 
    @class Parameter
    @brief A simple class to store information about a parameter, including the value, min, max, name and a short description
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef PARAMETER_H
#define PARAMETER_H

#include <vector>
#include <string>
#include <iostream>
using namespace std;

class Parameter 
{
public:
    Parameter();
    Parameter(float value, float min, float max);
    Parameter(string name, float value, float min, float max);
    Parameter(string name, float value, float min, float max, string desc);
    ~Parameter();
    
    float get() const;
    void set(float value);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    friend ostream& operator<< (ostream& output, const Parameter& p);
    friend ostream& operator<< (ostream& output, const vector<Parameter>& p);
    friend istream& operator>> (istream& input, Parameter& p);

private:
    string Name;
    float Value;
    float Min;
    float Max;
    string Description;
};

#endif
