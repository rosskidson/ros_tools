#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

//#include <Eigen/Core>

struct StampedPose
{
  StampedPose(const double time, const double x, const double y, const double z):
  time_(time),
  x_(x),
  y_(y),
  z_(z)
  {}

  double time_;
  double x_;
  double y_;
  double z_;

  double getLength() const
  {
    return sqrt(x_*x_ + y_*y_ + z_*z_);
  }

};

inline StampedPose operator-(StampedPose lhs, const StampedPose& rhs)
{
  StampedPose output(
          lhs.time_ - rhs.time_,
          lhs.x_ - rhs.x_,
          lhs.y_ - rhs.y_,
          lhs.z_ - rhs.z_);
  return output;
}

// read file
// takes a filename and reads entries and records them into a vector of StampedPose
void readFile(const std::string & filename, std::vector<StampedPose> & output_vector, const double add_offset = 0.0)
{
  std::ifstream in(filename.c_str());
  while(in) {
    std::string word;
    in >> word;
    if(in.peek()  == '\n')
    {
      double text_input[4];
      for(int i=0; i<4; i++)
        in >> std::setprecision(20) >> text_input[i];
      output_vector.push_back(StampedPose ((text_input[0]+add_offset), text_input[1], text_input[2], text_input[3]));
    }
  }
}

int
  main (int argc, char** argv)
{
  if(argc < 2)
  {
    std::cout << "please provide 1 file to align\n";
    exit(0);
  }
  std::string filename(argv[1]);

  std::vector<StampedPose> trajectory;
  readFile(filename, trajectory);

  double total_length(0.0);
  for(std::vector<StampedPose>::const_iterator itr = trajectory.begin(); 
      itr != trajectory.end() - 1; itr++)
  {
    const StampedPose & sp_t0 = *itr;
    const StampedPose & sp_t1 = *(itr+1);
    const StampedPose diff = sp_t1 - sp_t0;
    total_length += diff.getLength();
  }

  std::cout << "Total length " << total_length << "\n";

  return 0;
}

