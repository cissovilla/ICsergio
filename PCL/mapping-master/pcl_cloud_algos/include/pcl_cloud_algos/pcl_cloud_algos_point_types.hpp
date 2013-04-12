/*
 * Copyright (c) 2010, Hozefa Indorewala <indorewala@ias.in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

namespace pcl
{

  struct PointXYZINormalScanLine
  {
    PCL_ADD_POINT4D;
    float intensities;
    PCL_ADD_NORMAL4D;
    // represents the index of one XYZ triplet
    float index;
    // represents one complete 2D scan 
    float line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
  inline std::ostream& operator<<(std::ostream& os, const PointXYZINormalScanLine& p)
  {
    os << "("<<p.x<<","<<p.y<<","<<p.z<<" - "<<p.intensities<<" - "<<p.normal[0]<<","<<p.normal[1]<<","<<p.normal[2]<<" - "<<p.index<<" - "<<p.line<<")";
    return os;
  }

  struct ColorCHLACSignature981
  {
    float histogram[981];
  };
  inline std::ostream& operator << (std::ostream& os, const ColorCHLACSignature981& p)
  {
    for (int i = 0; i < 981; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 980 ? ", " : ")");
    return (os);
  }

  struct ColorCHLACSignature117
  {
    float histogram[117];
  };
  inline std::ostream& operator << (std::ostream& os, const ColorCHLACSignature117& p)
  {
    for (int i = 0; i < 117; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 116 ? ", " : ")");
    return (os);
  }

  struct GRSDSignature21
  {
    float histogram[21];
  };
  inline std::ostream& operator << (std::ostream& os, const GRSDSignature21& p)
  {
    for (int i = 0; i < 21; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 20 ? ", " : ")");
    return (os);
  }

  struct GRSDSignature325
  {
    float histogram[325];
  };
  inline std::ostream& operator << (std::ostream& os, const GRSDSignature325& p)
  {
    for (int i = 0; i < 325; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 324 ? ", " : ")");
    return (os);
  }

  struct PlusGRSDSignature110
  {
    float histogram[110];
  };
  inline std::ostream& operator << (std::ostream& os, const PlusGRSDSignature110& p)
  {
    for (int i = 0; i < 110; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 109 ? ", " : ")");
    return (os);
  }


struct PointNormalRADII
{
  PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])                                                              
  PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])                                                           
  float curvature, r_min, r_max;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
inline std::ostream& operator << (std::ostream& os, const PointNormalRADII& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << " - " << p.r_min << "," << p.r_max << ")";
  return (os);
}
  
}  // End namespace
