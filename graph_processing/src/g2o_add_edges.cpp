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

  int id1, id2;
  while(in) {
    bool next_edge_unity = false;
    std::string word;
    in >> word;
    if(word.compare(vertex_name) == 0)
    {
      out << std::endl << word << " ";
    }
    else if(word.compare(edge_name) == 0)
    {
      out << std::endl << word << " ";
      in >> id1;
      in >> id2;
      out << id1 << " " << id2 << " ";
      for(int i=0; i<7; i++)
      {
        in >> word;  //get past trans, quat
        out << word << " ";
      }

      double alpha;
      in >> alpha;
      out << alpha << " ";
      if(alpha < 1000)
        next_edge_unity=true;
      for(int i = 0; i<20; i++)
      {
        in >> word;
        out << word << " ";
      }
    }
    else
       out << word << " ";
    if(next_edge_unity)
      out << std::endl << edge_name << " " << id1 << " " << id2 << " 0 0 0 0 0 0 1 5000 0 0 0 0 0 5000 0 0 0 0 5000 0 0 0 5000 0 0 5000 0 5000 ";
  }


  out.close();
//  for (size_t i = 0; i < cloud.points.size (); ++i)
//    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
