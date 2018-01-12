#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

int main(int argc, char** argv)
{
    ifstream file(argv[1]);
    string line;
    for(int i = 0; i < 5; i++)
    {
	getline(file, line);
	stringstream stream(line);
	string s;
	getline(stream, s, ' ');
	cout << s << " ";

	getline(stream, s, ' ');
	cout << s << " ";
	getline(stream, s, ' ');
	cout << s << " ";
	getline(stream, s, ' ');
	cout << s << endl;
    }
    return 0;
}


