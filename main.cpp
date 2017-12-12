//Last update: 10/10/2014

#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "e57.h"

using namespace std;
using namespace e57;

int loadData(int argc, char **argv, vector<string> &files)
{
	std::string extension (".e57");
	// Suppose the first argument is the actual test model
    for (int i = 1; i < argc; ++i)
	{
		std::string fname = std::string (argv[i]);
		cout << fname <<endl;
		files.push_back(fname);	 
		return 1;
	}
    return 0;
}

static void savePtx (const std::string theFile, const pcl::PointCloud<pcl::PointXYZRGB>& theCloud)
{
  ofstream ofs (theFile);
  if (ofs.bad())
  {
    cout << "Error: Cannot open file " << theFile << " for writing!" << endl;
    return;
  }

  Eigen::Vector4f T = theCloud.sensor_origin_;
  Eigen::Matrix3f R = theCloud.sensor_orientation_.toRotationMatrix().inverse();

  ofs << theCloud.width << endl;
  ofs << theCloud.height << endl;
  ofs << T.x() << " " << T.y() << " " << T.z() << endl;
  ofs << R << endl;
  ofs << R(0,0) << " " << R(0,1) << " " << R(0,2) << " 0" << endl;
  ofs << R(1,0) << " " << R(1,1) << " " << R(1,2) << " 0" << endl;
  ofs << R(2,0) << " " << R(2,1) << " " << R(2,2) << " 0" << endl;
  ofs << T.x() << " " << T.y() << " " << T.z() << " 1" << endl;

  for (auto it = theCloud.begin(); it != theCloud.end(); ++it)
  {
    ofs << it->x << " " << it->y << " " << it->z << " 1 " << (int)it->r << " " << (int)it->g << " " << (int)it->b << endl;
  }
}

int main (int argc, char** argv)
{
  // Parse arguments
  vector<string> filenames;
  bool writeAscii = false;
  bool writePtx = false;
  bool defaultNames = false;
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = std::string(argv[i]);
    if (arg == "-ascii") 
    {
      writeAscii = true;
    }
    else if (arg == "-ptx") 
    {
      writePtx = true;
    }
    else if (arg == "-defnames")
    {
      defaultNames = true;
    }
    else 
    {
//      cout << arg << endl;
      filenames.push_back(arg);
    }
  }
  if (filenames.empty())
  {
    cout << "Error: no input fiels specified!" << endl;
    cout << "Syntax: " << endl;
    cout << argv[0] << " [-ascii] [-ptx] [-defnames] inpuf_file_1.e57 [imput_file_2.e57...]" << endl;
    return -1;
  }

  PtrXYZ cloud(new pcl::PointCloud<P_XYZ>);

  E57 e57;
  float scale_factor = 0;
  for (size_t i = 0; i < filenames.size(); ++i)
  {
    // Will be overwritten:
    int64_t scanCount = 1;

    for (int64_t scanIndex = 0; scanIndex < scanCount; ++scanIndex) {
      std::string scanname;
      if (e57.openE57(filenames.at(i), scanname, cloud, scale_factor, scanCount, scanIndex) == -1){
        cout << "Error reading file "  << filenames.at(i) << endl;
        return -1;
      }

      std::stringstream ss;
      if (defaultNames) // generate default names for scan files
      {
        ss << "Scan-" << i << "-" << scanIndex;
      }
      else { // use names of the scan for file 
        ss << scanname;
      }
      ss << (writePtx ? ".ptx" : ".pcd");
      cout << ss.str() << endl;

      // save the file
      if (writePtx)
      {
        savePtx (ss.str(), *cloud);
      }
      else if (writeAscii)
      {
        pcl::io::savePCDFileASCII(ss.str(), *cloud);
      }
      else
      {
        pcl::io::savePCDFileBinary(ss.str(), *cloud);
      }

/*
      //demonstration of writing down from PCD to E57
      if (e57.saveE57File("test.e57", cloud, scale_factor) == -1){
        cout << "Error saving in e57" << endl;
        return -1;
      }
*/
      cout << "********************* CONVERSION COMPLETED *********************" << endl;
      cout << "File: \t" << filenames.at(i) << endl;
      cout << "Cloud Size: \t" << cloud.get()->size() << endl;
      cout << "File Saved: \t" << ss.str() << endl;
      cout << "********************* CONVERSION COMPLETED *********************" << endl;
    }
  }

  cout << "Conversion completed" << endl;
  return 0;
}
