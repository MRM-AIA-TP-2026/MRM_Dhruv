// Exercise 1

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

char first = my_string.at(0);
char last = my_string.at(my_string.length()-1);

cout << first << " is the first character and " << last << " is the last character" << endl;

  //add code above this line
  
  return 0;
  
}

// Exercise 2

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

int length = my_string.length();
for (int i = 0; i < length; i++) {
    for (int j = 0; j < length; j++) {
      cout << my_string << " ";
    }
    cout << endl;
  }


  //add code above this line
  
  return 0;
  
}

// Exercise 3

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string original = (argv[1]);
  string modified;
  char ch;
  
  //add code below this line

 for (char c : original) {
    if (isupper(c)) {
      modified += 'u';
    } else if (islower(c)) {
      modified += 'l';
    } else {
      modified += '-';
    }
  }

  cout << original << endl;
  cout << modified << endl;


  //add code above this line
  
  return 0;
  
}

// Exercise 4

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

int len = my_string.length();
int mid = len / 2;
cout << my_string.substr(0, mid) << endl;
cout << my_string.substr(mid) << endl;

  //add code above this line
  
  return 0;
  
}

// Exercise 5

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string my_string = (argv[1]);
  
  //add code below this line

for (size_t i = 0; i < my_string.length(); i += 2){
  char temp = my_string[i];
  my_string[i] = my_string[i + 1];
  my_string[i + 1] = temp;
}
cout << my_string << endl;
  //add code above this line
  
  return 0;
  
}