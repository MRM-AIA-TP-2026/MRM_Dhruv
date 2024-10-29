// Exercise 1

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  int x = stoi(argv[1]);
  
  //add code below this line

if ((x>=0 && x<=25) || (x>=75 && x<=100)) {
  cout << to_string(x) + " is between 0 and 25 or 75 and 100" << endl;
}

  //add code above this line
  
  return 0;
  
}

// Exercise 2
#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  int x = stoi(argv[1]);
  
  //add code below this line
if (x%5==0) {
  cout << to_string(x) + " is divisible by 5" << endl;
}
else {
  cout << to_string(x) + " is not divisible by 5" << endl;
}


  //add code above this line
  
  return 0;
  
}


// Exercise 3

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  int x = stoi(argv[1]);
  
  //add code below this line
if ((x%5==0) && (x%2==0)) {
  cout << to_string(x) + " is divisible by 5 and even";
}
else {
  cout << to_string(x) + " is not divisible by 5 or it is odd";
}

  //add code above this line
  
  return 0;
  
}

// Exercise 4

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string x = argv[1];
  
  //add code below this line

if ((x=="red" || x=="blue" || x=="yellow")){
  cout << x + " is a primary color" << endl;
}
else {
  cout << x + " is not a primary color" << endl;
}
  //add code above this line
  
  return 0;
  
}
// Exercise 5

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string x = argv[1];
  
  //add code below this line

  if ((x=="a") || (x=="e") || (x=="i") || (x=="o") || (x=="u")){
    cout << x + " is a vowel" << endl;
  }
  else {
    cout << x + " is not a vowel" << endl;
  }

  //add code above this line
  
  return 0;
  
}