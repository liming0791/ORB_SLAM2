/**
* This file is part of CLATCH-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/CLATCH_SLAM2>
*
* CLATCH-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CLATCH-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CLATCH-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef CLATCHVOCABULARY_H
#define CLATCHVOCABULARY_H

#include"Thirdparty/DBoW2/DBoW2/FCLATCH.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FCLATCH::TDescriptor, DBoW2::FCLATCH>
  CLATCHVocabulary;

} //namespace CLATCH_SLAM

#endif // CLATCHVOCABULARY_H
