#ifndef __HAS__H_
#define __HAS__H_
namespace booang {
    using namespace std;
    template <typename T>
    class has_toString {
    private:
        typedef char Yes;
        typedef Yes No[2];

        template <typename U, U> struct really_has;
        template <typename C> static Yes& Test(really_has <string(C::*)() const, &C::toString>*);
        template <typename C> static Yes& Test(really_has <string(C::*)(), &C::toString>*);
        template <typename> static No& Test(...);
    public:
        static bool const value = sizeof(Test<T>(0)) == sizeof(Yes);
    };

};

#endif