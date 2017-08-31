#include <iostream>
#include <type_traits>

#include <boost/utility.hpp>
using namespace std;



// template<typename T, 
         // typename = typename std::enable_if<std::is_class<T>::value>::type>
// void f(T a){
// }

// template<typename T>
// class is_classu {
    // typedef char yes[1];
    // typedef char no [2];
    // template<typename C> static yes& test(int C::*); // C가 클래스인 경우 선택
    // template<typename C> static no&  test(...);      // 그 외의 경우 선택
  // public:
    // static bool const value = sizeof(test<T>(0)) == sizeof(yes);
// };

// class Widget {};

// template <unsigned a, unsigned b>
// typename boost::disable_if_c<a == b>::type foo()
// {
    // // foo<a+1,b>(); // <-- here we attempt to instantiate f<5,5> ...
                // // // but it gets disabled!

    // // and there is no other f visible here, it's defined just below
// }

// int main(){

  // Y<int> y1;
  // Y<string> y2;

  // f(y1);
  // // f(0);

  // y1.foo();
  // bool test1 = is_classu<int>::value; //false
  // bool test2 = is_classu<Widget>::value; //true
  // return 0;
// }



class foo;
class bar;

template<class T>
struct check
{
    template<class Q = T>
    typename std::enable_if<std::is_same<Q, bar>::value, bool>::type test()
    {
        return true;
    }

    template<class Q = T>
    typename std::enable_if<!std::is_same<Q, bar>::value, bool>::type test()
    {
        return false;
    }
};

template<typename T>
class Y {
public:
    T t;
    void etcFunctions() {}
    template<typename U = T>
    typename std::enable_if<std::is_integral<U>::value, void>::type foo() {
        t = t / t;
    }
    template<typename U = T>
    typename std::enable_if<!std::is_integral<U>::value, void>::type foo();
};

int main()
{
    Y<int> y1;
    Y<string> y2;
    y1.foo();

    return 0;
}
