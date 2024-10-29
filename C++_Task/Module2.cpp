// Exercise 1
/*
1.) Fixing compilation error (a was declared as string. Changed it to int.)
2.) Fixing run-time error (Desired output was 17 so changed the value of b from 9 to 6.)
*/
#include <iostream>
using namespace std;

int main() {
  
  //fix the code below this line

  int a = 5;
  int b = 6;

  //fix the code above this line
  
  cout << (a * 3 + b - 8 / 2) << endl;
  
  return 0;
  
}

// Exercise 2
/*
Using ONLY numbers and operators, print an expression that evaluates to false.
Make sure to use cout << boolalpha << in your code. 
Otherwise, the system will print 0 or 1 instead of false or true.
*/
#include <iostream>
using namespace std;

int main() {
  
  //add code below this line

cout << boolalpha << ((5==5) && (5!=5)) << endl;

  //add code above this line
  
  return 0;
  
}

// Exercise 3
// Write a program that outputs Hello world.

#include <iostream>
using namespace std;

int main() {
  
  //add code below this line
string a="Hello";
string b=" world";
cout << a+b << endl;


  //add code above this line
  
  return 0;
  
}

// Exercise 4
/*
Write a program that divides 7 by 2 
and prints out the result (3.5) without using the printf() command.
*/
#include <iostream>
using namespace std;

int main() {
  
  //add code below this line
double a=7;
double b=2;
cout << a/b << endl;


  //add code above this line
  
  return 0;
  
}

//Exercise 5
// Debugging the adder

#include <iostream>
using namespace std;

int main() {
  
  int num1;
  int num2;
  cout << "Type the first whole number and then press Enter or Return: ";
  cin >> num1;
  cout << "Type the second whole number and then press Enter or Return: ";
  cin >>num2;
  
  //fix the code below this line

  int sum = num1 + num2;
  cout << ( to_string(num1) + " + " + to_string(num2) + " = " + to_string(sum)) << endl;

  //fix the code above this line
  
  return 0;
}