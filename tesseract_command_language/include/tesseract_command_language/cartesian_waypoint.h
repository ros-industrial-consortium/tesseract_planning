/**
 * @file cartesian_waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class CartesianWaypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianWaypoint() = default;

  CartesianWaypoint(const tinyxml2::XMLElement& xml_element)
  {
    if (xml_element.Attribute("xyz") == nullptr && xml_element.Attribute("rpy") == nullptr &&
        xml_element.Attribute("wxyz") == nullptr)
      throw std::runtime_error("CartesianWaypoint: Missing Required Attributes");

    waypoint = Eigen::Isometry3d::Identity();

    std::string xyz_string, rpy_string, wxyz_string;
    tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(&xml_element, "xyz", xyz_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("CartesianWaypoint: Error parsing attribute xyz.");

    if (status != tinyxml2::XML_NO_ATTRIBUTE)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, xyz_string, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
        throw std::runtime_error("CartesianWaypoint: Error parsing attribute xyz string.");

      double x{ 0 }, y{ 0 }, z{ 0 };
      // No need to check return values because the tokens are verified above
      tesseract_common::toNumeric<double>(tokens[0], x);
      tesseract_common::toNumeric<double>(tokens[1], y);
      tesseract_common::toNumeric<double>(tokens[2], z);

      translation() = Eigen::Vector3d(x, y, z);
    }

    if (xml_element.Attribute("wxyz") == nullptr)
    {
      status = tesseract_common::QueryStringAttribute(&xml_element, "rpy", rpy_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("CartesianWaypoint: Error parsing attribute rpy.");

      if (status != tinyxml2::XML_NO_ATTRIBUTE)
      {
        std::vector<std::string> tokens;
        boost::split(tokens, rpy_string, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
          throw std::runtime_error("CartesianWaypoint: Error parsing attribute rpy string.");

        double r{ 0 }, p{ 0 }, y{ 0 };
        // No need to check return values because the tokens are verified above
        tesseract_common::toNumeric<double>(tokens[0], r);
        tesseract_common::toNumeric<double>(tokens[1], p);
        tesseract_common::toNumeric<double>(tokens[2], y);

        Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond rpy = yawAngle * pitchAngle * rollAngle;

        linear() = rpy.toRotationMatrix();
      }
    }
    else
    {
      status = tesseract_common::QueryStringAttribute(&xml_element, "wxyz", wxyz_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("CartesianWaypoint: Error parsing attribute wxyz.");

      if (status != tinyxml2::XML_NO_ATTRIBUTE)
      {
        std::vector<std::string> tokens;
        boost::split(tokens, wxyz_string, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.size() != 4 || !tesseract_common::isNumeric(tokens))
          throw std::runtime_error("CartesianWaypoint: Error parsing attribute wxyz string.");

        double qw{ 0 }, qx{ 0 }, qy{ 0 }, qz{ 0 };
        // No need to check return values because the tokens are verified above
        tesseract_common::toNumeric<double>(tokens[0], qw);
        tesseract_common::toNumeric<double>(tokens[1], qx);
        tesseract_common::toNumeric<double>(tokens[2], qy);
        tesseract_common::toNumeric<double>(tokens[3], qz);

        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();

        linear() = q.toRotationMatrix();
      }
    }

    // Parse tolerances
    const tinyxml2::XMLElement* upper_tolerance_element = xml_element.FirstChildElement("UpperTolerance");
    const tinyxml2::XMLElement* lower_tolerance_element = xml_element.FirstChildElement("LowerTolerance");

    if (upper_tolerance_element && lower_tolerance_element)
    {
      std::string upper_tolerance_string, lower_tolerance_string;

      status = tesseract_common::QueryStringText(upper_tolerance_element, upper_tolerance_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("CartesianWaypoint: Error parsing UpperTolerance string");

      status = tesseract_common::QueryStringText(lower_tolerance_element, lower_tolerance_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("CartesianWaypoint: Error parsing LowerTolerance string");

      std::vector<std::string> upper_tolerance_tokens, lower_tolerance_tokens;
      if (!upper_tolerance_string.empty() && !lower_tolerance_string.empty())
      {
        boost::split(upper_tolerance_tokens, upper_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);
        boost::split(lower_tolerance_tokens, lower_tolerance_string, boost::is_any_of(" "), boost::token_compress_on);
      }

      if (!tesseract_common::isNumeric(upper_tolerance_tokens))
        throw std::runtime_error("CartesianWaypoint: UpperTolerance are not all numeric values.");

      if (!tesseract_common::isNumeric(lower_tolerance_tokens))
        throw std::runtime_error("CartesianWaypoint: LowerTolerance are not all numeric values.");

      upper_tolerance.resize(static_cast<Eigen::Index>(upper_tolerance_tokens.size()));
      lower_tolerance.resize(static_cast<Eigen::Index>(lower_tolerance_tokens.size()));
      for (long i = 0; i < static_cast<long>(upper_tolerance_tokens.size()); ++i)
      {
        tesseract_common::toNumeric<double>(upper_tolerance_tokens[static_cast<std::size_t>(i)], upper_tolerance[i]);
        tesseract_common::toNumeric<double>(lower_tolerance_tokens[static_cast<std::size_t>(i)], lower_tolerance[i]);
      }
    }
  }

  int getType() const { return static_cast<int>(WaypointType::CARTESIAN_WAYPOINT); }

  void print(const std::string& prefix = "") const
  {
    std::cout << prefix << "Cart WP: xyz=" << this->translation().x() << ", " << this->translation().y() << ", "
              << this->translation().z() << std::endl;
    // TODO: Add rotation
  }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
    tinyxml2::XMLElement* xml_waypoint = doc.NewElement("Waypoint");
    xml_waypoint->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_cartesian_waypoint = doc.NewElement("CartesianWaypoint");
    std::stringstream xyz_string;
    xyz_string << translation().format(eigen_format);
    xml_cartesian_waypoint->SetAttribute("xyz", xyz_string.str().c_str());

    std::stringstream wxyz_string;
    Eigen::Quaterniond q(linear());
    wxyz_string << Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()).format(eigen_format);

    xml_cartesian_waypoint->SetAttribute("wxyz", wxyz_string.str().c_str());
    xml_waypoint->InsertEndChild(xml_cartesian_waypoint);

    // Write upper tolerance
    {
      std::stringstream upper_string;
      upper_string << upper_tolerance.format(eigen_format);

      tinyxml2::XMLElement* xml_upper = doc.NewElement("UpperTolerance");
      xml_upper->SetText(upper_string.str().c_str());
      xml_cartesian_waypoint->InsertEndChild(xml_upper);
    }

    // Write lower tolerance
    {
      std::stringstream lower_string;
      lower_string << lower_tolerance.format(eigen_format);

      tinyxml2::XMLElement* xml_lower = doc.NewElement("LowerTolerance");
      xml_lower->SetText(lower_string.str().c_str());
      xml_cartesian_waypoint->InsertEndChild(xml_lower);
    }

    return xml_waypoint;
  }

  /////////////////////
  // Eigen Container //
  /////////////////////

  using ConstLinearPart = Eigen::Isometry3d::ConstLinearPart;
  using LinearPart = Eigen::Isometry3d::LinearPart;
  using ConstTranslationPart = Eigen::Isometry3d::ConstTranslationPart;
  using TranslationPart = Eigen::Isometry3d::TranslationPart;

  ////////////////////////
  // Eigen Constructors //
  ////////////////////////

  // This constructor allows you to construct from Eigen expressions
  template <typename OtherDerived>
  CartesianWaypoint(const Eigen::MatrixBase<OtherDerived>& other) : waypoint(other)
  {
  }

  CartesianWaypoint(const Eigen::Isometry3d& other) : waypoint(other) {}

  ///////////////////
  // Eigen Methods //
  ///////////////////

#ifndef SWIG
  /** @returns a read-only expression of the linear part of the transformation */
  inline ConstLinearPart linear() const { return waypoint.linear(); }

  /** @returns a writable expression of the linear part of the transformation */
  inline LinearPart linear() { return waypoint.linear(); }

  /** @returns a read-only expression of the translation vector of the transformation */
  inline ConstTranslationPart translation() const { return waypoint.translation(); }

  /** @returns a writable expression of the translation vector of the transformation */
  inline TranslationPart translation() { return waypoint.translation(); }
#else   // SWIG
  Eigen::Matrix3d linear() const;
  Eigen::Matrix3d linear();
  Eigen::Vector3d translation() const;
  Eigen::Vector3d translation();
#endif  // SWIG

  /** @returns true if two are approximate */
  inline bool isApprox(const Eigen::Isometry3d& other, double prec = 1e-12) { return waypoint.isApprox(other, prec); }

  /////////////////////
  // Eigen Operators //
  /////////////////////

  // This method allows you to assign Eigen expressions to MyVectorType
  template <typename OtherDerived>
  inline CartesianWaypoint& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = other;
    return *this;
  }

  template <typename OtherDerived>
  inline CartesianWaypoint& operator*=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = waypoint * other;
    return *this;
  }

  template <typename OtherDerived>
  inline CartesianWaypoint operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    CartesianWaypoint cwp = *this;
    cwp.waypoint = cwp.waypoint * other;
    return cwp;
  }

  inline CartesianWaypoint& operator=(const Eigen::Isometry3d& other)
  {
    waypoint = other;
    return *this;
  }

  inline CartesianWaypoint& operator*=(const Eigen::Isometry3d& other) { return *this = waypoint * other; }

  inline CartesianWaypoint operator*(const Eigen::Isometry3d& other) const
  {
    CartesianWaypoint cwp = *this;
    cwp.waypoint = cwp.waypoint * other;
    return cwp;
  }

  //////////////////////////
  // Implicit Conversions //
  //////////////////////////

  /** @return Implicit Conversions to read-only Eigen::Isometry3d */
  inline operator const Eigen::Isometry3d&() const { return waypoint; }

  /** @return Implicit Conversions to writable Eigen::Isometry3d */
  inline operator Eigen::Isometry3d&() { return waypoint; }

  //////////////////////////////////
  // Cartesian Waypoint Container //
  //////////////////////////////////

  /** @brief The Cartesian Waypoint */
  Eigen::Isometry3d waypoint{ Eigen::Isometry3d::Identity() };
  /** @brief Distance below waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()) */
  Eigen::VectorXd lower_tolerance;
  /** @brief Distance above waypoint that is allowed. Should be size = 6. First 3 elements are dx, dy, dz. The last 3
   * elements are angle axis error allowed (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle())*/
  Eigen::VectorXd upper_tolerance;

  /**
   * @brief Returns true if waypoint is toleranced
   * @return True if waypoint is toleranced
   */
  bool isToleranced() const
  {
    // Check if they are empty
    if (lower_tolerance.size() == 0 || upper_tolerance.size() == 0)
      return false;

    // Check if they are the same
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    if ((lower_tolerance.array() > max_diff).any())
      throw std::runtime_error("CartesianWaypoint: lower tolerance was provided but must be <= 0,");

    if ((upper_tolerance.array() < -max_diff).any())
      throw std::runtime_error("CartesianWaypoint: upper tolerance was provided but must be >= 0,");

    return !tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, upper_tolerance, max_diff);
  }
  bool operator==(const CartesianWaypoint& rhs) const
  {
    static auto max_diff = static_cast<double>(std::numeric_limits<float>::epsilon());

    bool equal = true;
    equal &= waypoint.isApprox(rhs.waypoint);
    equal &= tesseract_common::almostEqualRelativeAndAbs(lower_tolerance, rhs.lower_tolerance, max_diff);
    equal &= tesseract_common::almostEqualRelativeAndAbs(upper_tolerance, rhs.upper_tolerance, max_diff);
    return equal;
  }
  bool operator!=(const CartesianWaypoint& rhs) const { return !operator==(rhs); }

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_NVP(waypoint);
    ar& BOOST_SERIALIZATION_NVP(upper_tolerance);
    ar& BOOST_SERIALIZATION_NVP(lower_tolerance);
  }
};

}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_waypoint_type(CartesianWaypoint)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
