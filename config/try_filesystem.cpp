#include <iostream>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>

using std::cout;
using namespace boost::filesystem;

int main(int argc, char** argv)
{
    path p(argv[1]);

    std::vector<std::string> names(0);

    for(directory_entry & x: directory_iterator(p))
    {
	std::string filename = x.path().filename().string();
	//cout << x.path() << " " << std::endl;
	if(extension(filename) == ".png")
	     names.push_back(x.path().filename().string());
    }

    std::sort(names.begin(), names.end());
    for(auto elem : names)
	cout << elem <<std::endl;
    return 0;
}


