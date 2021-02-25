/**
 * @file serialize.h
 * @brief Provide methods for serializing instructions to xml and deserialization
 *
 * @author Levi Armstrong
 * @date August 17, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COMMAND_LANGUAGE_SERIALIZE_H
#define TESSERACT_COMMAND_LANGUAGE_SERIALIZE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
template <typename SerializableType>
std::shared_ptr<tinyxml2::XMLDocument> toXMLDocument(const SerializableType& input);

template <typename SerializableType>
bool toXMLFile(const SerializableType& input, const std::string& file_path);

template <typename SerializableType>
std::string toXMLString(const SerializableType& input);
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_SERIALIZE_H
