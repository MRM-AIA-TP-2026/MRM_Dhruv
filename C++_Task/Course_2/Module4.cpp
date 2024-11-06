// Exercise 1
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line
 try {
  ifstream file;
  string read;
  int lines = 0;
  int chars = 0;
  file.open(path);
  if (!file) {
    throw runtime_error("File failed to open.");
  }
  while (getline(file, read)) {
    lines++;
    chars += read.length();
  }
  file.close();
  cout << lines << " line(s)" << endl;
  cout << chars << " character(s)"; 
}
  
catch (exception& e) {
  cerr << e.what() << endl;
}

  
  //add code above this line
  
  return 0;
  
}

//Exercise 2
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line
 vector<string> data;

 try {
   ifstream file;
   string read;
   file.open(path);
  if (!file) {
    throw runtime_error("File failed to open.");   }
  while (getline(file, read)) {
     stringstream ss(read);
     while (getline(ss, read, ',')) {
       data.push_back(read);
     }
   }
   file.close();
 }
    catch (exception& e) {
 cerr << e.what() << endl;
 }
   
 int col1 = 0;
 int col2 = 0;
 int col3 = 0;
 int col4 = 0;
 for (int i = 0; i < data.size(); i++) {
   if (i == 0 || i == 4 | i == 8) {
     col1 += stoi(data.at(i));
   }
   if (i == 1 || i == 5 | i == 9) {
     col2 += stoi(data.at(i));
   }
   if (i == 2 || i == 6 | i == 10) {
     col3 += stoi(data.at(i));
   }
   if (i == 3 || i == 7 | i == 11) {
     col4 += stoi(data.at(i));
  }
}
  
 cout << col1 / 3 << " ";
 cout << col2 / 3 << " ";
 cout << col3 / 3 << " ";
cout << col4 / 3;


  //add code above this line
  
  return 0;
  
}

// Exercise 3
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line

 vector<string> data;

try {
  ifstream file;
  string read;
  file.open(path);
  if (!file) {
    throw runtime_error("File failed to open.");
  }
  while (getline(file, read)) {
    stringstream ss(read);
    while (getline(ss, read)) {
      data.push_back(read);
    }
  }
  file.close();
}
  
catch (exception& e) {
  cerr << e.what() << endl;
}
  
for (int i = data.size() - 1; i >= 0; i--) {
  cout << data.at(i) << endl;
}



  //add code above this line
  
  return 0;
  
}

// Exercise 4

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line
ifstream file(path);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
        return 1;
    }

    string line, name, occupation;
    int age;
    string oldestPerson;
    int maxAge = -1;  // Initialize with an impossible age

    while (getline(file, line)) {
        stringstream ss(line);
        string age_str;

        // Read name, age, and occupation from the line
        getline(ss, name, '\t');
        getline(ss, age_str, '\t');
        getline(ss, occupation, '\t');

        // Convert age from string to int
        age = stoi(age_str);

        // Check if this person is older than the current oldest
        if (age > maxAge) {
            maxAge = age;
            oldestPerson = name;
        }
    }

    if (maxAge != -1) {
        cout << "The oldest person is " << oldestPerson << "." << endl;
    } else {
        cout << "No data found." << endl;
    }


  //add code above this line
  
  return 0;
  
}


// Exercise 5
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char** argv) {

////////// DO NOT EDIT! //////////
  string path = argv[1];        //
//////////////////////////////////  
  
  //add code below this line
vector<string> data;

try {
  ifstream file;
  string read;
  file.open(path);
  if (!file) {
    throw runtime_error("File failed to open.");
  }
  while (getline(file, read)) {
    stringstream ss(read);
    while (getline(ss, read, ',')) {
      data.push_back(read);
    }
  }
  file.close();
}
  
catch (exception& e) {
  cerr << e.what() << endl;
}
  
string cities;
cout << "The following cities are in the Southern Hemisphere: ";
  
for (int i = 6; i < data.size(); i+=4) {
  if (stoi(data.at(i)) < 0) {
    cities += (data.at(i - 2) + ", ");
  }
}
  
cities.pop_back();
cities.pop_back();
cities += ".";
  
cout << cities;


  //add code above this line
  
  return 0;
  
}