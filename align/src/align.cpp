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
};

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

// align vectors
// iterates through the first vector and for each entry finds the closest corresponding StampedPose in time
// returns a vector of aligned StampedPose pairs
void alignVectors(const std::vector<StampedPose> & vector_1,
                  const std::vector<StampedPose> & vector_2,
                  std::vector<std::pair<StampedPose, StampedPose> > & aligned_vector)
{
  for(std::vector<StampedPose>::const_iterator itr = vector_1.begin(); itr != vector_1.end(); itr++)
  {
    const double & ref_time = itr->time_;
    double smallest_time = 1e6;
    std::vector<StampedPose>::const_iterator closest_pose_itr;
    for(std::vector<StampedPose>::const_iterator itr_inner = vector_2.begin(); itr_inner != vector_2.end(); itr_inner++)
    {
      const double & search_time = itr_inner->time_;
      const double time_diff = fabs(ref_time - search_time);
      if(time_diff < smallest_time)
      {
        smallest_time = time_diff;
        closest_pose_itr = itr_inner;
      }
    }
    std::cout << "time diff " << smallest_time << "\n";
    aligned_vector.push_back(std::make_pair<StampedPose, StampedPose>(*itr, *closest_pose_itr));
  }
}

// print aligned vector
// takes a vector of aligned StampedPose pairs and prints to file
void printAlignedVector(const std::string & filename, const std::vector<std::pair<StampedPose, StampedPose> > & aligned_vector)
{
  std::ofstream out;
  out.open (filename.c_str());
  for(std::vector<std::pair<StampedPose, StampedPose> >::const_iterator itr = aligned_vector.begin(); itr != aligned_vector.end(); itr++)
  {
    const StampedPose & pose_2 = itr->first;
    const StampedPose & pose_1 = itr->second;
    out << pose_1.x_ << " " << pose_1.y_ << " " << pose_1.z_ << " "  <<
           pose_2.x_ << " " << pose_2.y_ << " " << pose_2.z_ << "\n";
  }
  out.close();
}

int
  main (int argc, char** argv)
{
  if(argc < 3)
  {
    std::cout << "please provide 2 files to align\n";
    std::cout << "optional third argument time offset to be applied to the first file\n";
    exit(0);
  }
  std::string filename_a(argv[1]);
  std::string filename_b(argv[2]);
  double time_offset = 0.0;
  if(argc == 4)
  {
    time_offset = atof(argv[3]);
    std::cout << " read in time offset " << time_offset << ".  Will be applied to first file\n";
  }

  std::vector<StampedPose> trajectory_1, trajectory_2;
  readFile(filename_a, trajectory_1, time_offset);
  readFile(filename_b, trajectory_2);

  std::cout << "t1 " << trajectory_1.size() << "\n";
  std::cout << "t2 " << trajectory_2.size() << "\n";

  std::vector<std::pair<StampedPose, StampedPose> > aligned_vector;
  if(trajectory_1.size() > trajectory_2.size())
    alignVectors(trajectory_2, trajectory_1, aligned_vector);
  else
    alignVectors(trajectory_1, trajectory_2, aligned_vector);

  printAlignedVector("aligned_output.txt", aligned_vector);

}

