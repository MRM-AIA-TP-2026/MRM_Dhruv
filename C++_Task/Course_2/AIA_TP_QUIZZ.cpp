
// Q1
#include <iostream>
using namespace std;

int main() {
    double pAmt, discount = 0.00;
    cout << "Enter the amount: ";
    cin >> pAmt;
    if (pAmt>= 1000) {
        discount = 0.20;
    } 
    else if (pAmt >= 500) {
        discount = 0.15;
    } 
    else if (pAmt >= 100) {
        discount = 0.10;
    } 
    else {
        discount = 0.0;
    }
    double fAmt = pAmt * (1 - discount);
    cout << "The final amount after discount: " << fAmt << endl;

    return 0;
}


// Q2

#include <iostream>
using namespace std;

int main() {
    int Num;
    cout << "Enter the number of rows: ";
    cin >> Num;
    int var = 1;
    for (int i = 1; i <= Num; ++i) {
        for (int j = 1; j <= i; ++j) {
            cout << var << " ";
            var++;
        }
        cout << endl;
    }
    return 0;
}




// Q3

#include <iostream>
using namespace std;

int main() {
    int Num;
    cout << "Enter the number of primes to print: ";
    cin >> Num;
    int count = 0;
    int var = 2;
    cout << "The prime numbers are: ";
    while (count < Num) {
        int flag = 1;
        for (int i = 2; i * i <= var; ++i) {
            if (var % i == 0) {
                flag = 0;
                break;
            }
        }
        if (flag == 1) {
            cout << var << " ";
            count++;
        }
        var++;
    }
    cout << endl;

    return 0;
}
