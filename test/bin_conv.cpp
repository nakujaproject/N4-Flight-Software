#include <iostream>
#include <bitset>
#include <string>
int bit_length = 8;

int main() {

    std::bitset<8> y = 106;
    std::cout << y << "\n";

    std::string s = "1101011";
    std::string t = "101011";

    //s.insert(0,2, '0');

    std::cout << s.length() << std::endl;
    std::cout << t.length() << std::endl;

    std::cout << "After inserting.." << std::endl;
    s = s.insert(0, bit_length-s.length(), '0' );
    t = t.insert(0, bit_length-t.length(), '0' );

    std::cout << s << std::endl;
    std::cout << t << std::endl;

    return 0;

}