// Exercise 1
/*
#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  int a = atoi((argv[1]));
  int b = atoi((argv[2]));
  int *p1 = &a;
  int *p2 = &b;
  
  //add code below this line
  
  if (*p1 > *p2){
    cout << "The larger number is " << *p1 << endl;
  }
  else if (*p1 < *p2){
    cout << "The larger number is " << *p2 << endl;
  }
  else {
    cout << "Neither number is larger" << endl;
  }
  //add code above this line
  
  return 0;
  
}

*/


// Exercise 2

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  double a;
  double b;
  double *p1;
  double *p2;
  
  cout << "Enter the first number: " << endl;
  cin >> a;
  if (!cin.good()) {
    cout << "invalid" << endl;
    exit(0);
  }
  cout << "Enter the second number: " << endl;
  cin >> b;
  if (!cin.good()) {
    cout << "invalid" << endl;
    exit(0);
  }
  
  // your code goes below
  
  p1 = &a;
  p2 = &b;

  cout << "The sum is " ;
  // your code goes above
  
  cout << *p1 + *p2 << endl;
  
  return 0;
  
}