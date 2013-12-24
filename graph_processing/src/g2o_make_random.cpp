#include <iostream>
#include <fstream>

#include <Eigen/Core>

int
  main (int argc, char** argv)
{
  if(argc < 2)
  {
    std::cout << "please provide g2o filename to be converted\n";
    exit(0);
  }
  std::string filename(argv[1]);

  const std::string vertex_name = "G2O_VERTEX_SE3";
  const std::string edge_name = "G2O_EDGE_SE3";
  const std::string reproj_edge_name = "G2O_EDGE_PROJECT_PSI2UVU";
  const std::string landmark_name = "G2O_VERTEX_POINT_XYZ";
  std::ifstream in(filename.c_str());

  std::ofstream out;
  out.open ("g2o_out.g2o");

  int id1, id2;
  while(in) {
    bool next_edge_unity = false;
    std::string word;
    in >> word;
    if(word.compare(edge_name) == 0)
    {
      out << std::endl << word << " ";
    }
    else if(word.compare(reproj_edge_name) == 0)
    {
      out << std::endl << word << " ";
    }
    else if(word.compare(landmark_name) == 0)
    {
      out << std::endl << word << " ";
    }
    else if(word.compare(vertex_name) == 0)
    {
      out << std::endl << word << " ";
      in >> id1;
      out << id1 << " ";
      for(int i=0; i<3; i++)
      {
        in >> word;  //get past trans, quat
        double rand_number = (double)(rand() % 1000)/10.0;
        out << rand_number << " ";
      }
      for(int i=0; i<4; i++)
      {
        in >> word;  //get past trans, quat
        double rand_number = (double)(rand() % 1000)/1000.0;
        out << rand_number << " ";
      }

    }
    else
       out << word << " ";
  }


  out.close();

  return (0);
}
