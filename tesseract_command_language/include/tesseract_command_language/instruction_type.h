/**
 * @file instruction_type.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H

namespace tesseract_planning
{
class Instruction;

enum class InstructionType : int
{
  // Everything before must be a Null Instruction
  NULL_INSTRUCTION = 0,

  // Everything before must be a motion plan Instruction
  PLAN_INSTRUCTION = 10,

  // Everything before must be a motion Instruction
  MOVE_INSTRUCTION = 20,

  // Everything before must be a composite Instruction
  COMPOSITE_INSTRUCTION = 30,

  // Everything before must be a I/O Instruction
  IO_INSTRUCTION = 40,

  // Everything before must be a Analog Instruction
  ANALOG_INSTRUCTION = 50,

  // Everything before must be a variable Instruction
  VARIABLE_INSTRUCTION = 60,

  // Everything before must be a comment Instruction
  COMMENT_INSTRUCTION = 70,

  // Everything before must be a wait Instruction
  WAIT_INSTRUCTION = 80,

  // Everything before must be a timer Instruction
  TIMER_INSTRUCTION = 90,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

bool isCommentInstruction(const Instruction& instruction);

bool isVariableInstruction(const Instruction& instruction);

bool isAnalogInstruction(const Instruction& instruction);

bool isIOInstruction(const Instruction& instruction);

bool isCompositeInstruction(const Instruction& instruction);

bool isMoveInstruction(const Instruction& instruction);

bool isPlanInstruction(const Instruction& instruction);

bool isNullInstruction(const Instruction& instruction);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
