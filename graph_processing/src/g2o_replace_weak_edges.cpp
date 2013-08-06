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
  std::ifstream in(filename.c_str());

  std::ofstream out;
  out.open ("g2o_out.g2o");

  while(in) {
    std::string word;
    in >> word;
    if(word.compare(vertex_name) == 0)
    {
      out << std::endl << word << " ";
    }
    else if(word.compare(edge_name) == 0)
    {
      out << std::endl << word << " ";
      for(int i=0; i<9; i++)
      {
        in >> word;  //get rid of the ids, trans, quat
        out << word << " ";
      }

      double alpha;
      in >> alpha;
      if(alpha < 1000)
      {
        out << "50000 0 0 0 0 0 50000 0 0 0 0 50000 ";
        for(int i = 0; i<11; i++)
          in >> word;
      }
      else
      {
        out << alpha << " ";
        for(int i = 0; i<11; i++)
        {
          in >> word;
          out << word << " ";
        }
      }
    }
    else
       out << word << " ";
  }


  out.close();
//  for (size_t i = 0; i < cloud.points.size (); ++i)
//    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
