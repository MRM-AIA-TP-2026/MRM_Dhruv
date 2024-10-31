// Exercise 1
// Turtule commands
// To draw a triangle using for loop

////////// DO NOT EDIT HEADER! //////////
#include <iostream>                    //
#include "CTurtle.hpp"                 //
#include "CImg.h"                      //
using namespace cturtle;               //
using namespace std;                   //
/////////////////////////////////////////

int main(int argc, char** argv) {
  
  TurtleScreen screen(400, 300); //You may edit the dimensions to fit your window
  Turtle tina(screen);
  tina.speed(TS_SLOWEST);
  for (int i=0;i<3;i++){
    tina.forward(100);
    tina.left(120);
  }
  
  //add code below this line



  //add code above this line
  
  screen.exitonclick();
  return 0;
  
}



// Exercise 2
// To print a given string ten times

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  string x = argv[1];
  
  //add code below this line
for (int i=0;i<10;i++){
  cout << x << endl;
}


  //add code above this line
  
  return 0;
  
}

// Exercise 3
// To find the sum of all numbers between two limits (inclusive)

#include <iostream>
using namespace std;

int main(int argc, char** argv) {
  
  int a = stoi(argv[1]);
  int b = stoi(argv[2]);
  
  if (a > b) {
    int c = b;
    b = a;
    a = c;
  }
  int sum=0;
  for (a;a<=b;a++){
    sum+=a;
  }
  cout << sum << endl;
  //add code below this line



  //add code above this line
  
  return 0;
  
}

// Exercise 4 
// To add break statements in correct place

#include <iostream>
using namespace std;

int main() {

  for (int i = 100; i <= 100; i--) {
    if (i == 0) {
      cout << "Print me!" << endl;
      break;
    }
    
    else {
      
      while (true) {
        break;
        i++;
        cout << "Don't print me!" << endl;
        
      }
      
    }
    
  }

  return 0;

}

// Exercise 5
// Print a given pattern


#include <iostream>
using namespace std;

int main() {
  
  //add code below this line
for (int i=1;i<=5;i++){
  for (int j=5-i;j>0;j--){
    cout << '.' ;
  }
  cout << i << endl;
}


  //add code above this line
  
  return 0;
  
}

