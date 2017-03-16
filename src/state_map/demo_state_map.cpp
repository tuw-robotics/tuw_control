#include <tuw_control/state_map/state_map.hpp>

#include <iostream>

using namespace tuw;
using namespace std;


using TestDepth1  = StateMapVector<double, double>;
using TestDepth0  = StateMapVector<double, TestDepth1>;

using TestDepth1Arr  = StateMapArray<double, double, 4>;
using TestDepth0Arr  = StateMapArray<double, TestDepth1Arr, 3>;

using TestDepth0ArrVec  = StateMapArray<double, TestDepth1, 3>;
using TestDepth0VecArr  = StateMapVector<double, TestDepth1Arr>;

using TestTuple   = StateMapTuple<double, TestDepth1Arr, TestDepth0Arr, TestDepth0Arr>;
using TestTuple2  = StateMapTuple<double, TestTuple, TestDepth0Arr, TestDepth1Arr>;

int main() {
    
    TestTuple tup;
    cout<<"isAllStaticUnder="<<tup.MapSize<<endl;
    TestTuple2 tup2;
    cout<<"isAllStaticUnder="<<tup2.MapSize<<endl;
    TestDepth0Arr arr;
    cout<<"isAllStaticUnder="<<arr.MapSize<<endl;
    Eigen::Vector4d hellu(1.,2.,3.,4.);
    arr.sub(1).data() = hellu;
    cout<<arr.data().transpose()<<endl<<endl;
    cout<<arr.sub(0).data().transpose()<<endl<<endl;
    cout<<arr.sub(1).data().transpose()<<endl<<endl;
    cout<<arr.sub(2).data().transpose()<<endl<<endl;
    arr.sub<0>();
    TestDepth0ArrVec arrvec;
    cout<<"isAllStaticUnder="<<arrvec.MapSize<<endl;
    arrvec.sub(1).subResize(4);
    arrvec.sub(1).data() = hellu;
    cout<<arrvec.data().transpose()<<endl<<endl;
    TestDepth0VecArr vecarr;
    cout<<"isAllStaticUnder="<<vecarr.MapSize<<endl;
    vecarr.subResize(4);
    vecarr.sub(1).data() = hellu;
    cout<<vecarr.data().transpose()<<endl<<endl;
    
    TestDepth0 DerivedObjectTestDerived, DerivedObjectTestCTRP, DerivedObjectVirtual;
    
    TestDepth0                    testDerived     = DerivedObjectTestDerived;
    StateMapBaseCRTP<TestDepth0>& testBaseCRTP    = DerivedObjectTestCTRP;
    StateMapBaseVirt<double>&     testBaseVirtual = DerivedObjectVirtual;
    
    
    cout<<endl<<"Starting derived Test"<<endl<<endl;
    testDerived.subResize(5);
    testDerived.sub(1).subResize(5); 
    for(int i = 0; i < testDerived.sub(1).data().rows(); ++i) { testDerived.sub(1).data()(i) = 9-i; }
    testDerived.sub(1).subResize(3); 
    testDerived.sub(0).subResize(2);
    for(int i = 0; i < testDerived.sub(0).data().rows(); ++i) { testDerived.sub(0).data()(i) = 1+i; }
    cout<<endl<<"Starting baseCRTP Test"<<endl<<endl;
    testBaseCRTP.subResize(5);
    testBaseCRTP.sub(1).subResize(5); 
    for(int i = 0; i < testBaseCRTP.sub(1).data().rows(); ++i) { testBaseCRTP.sub(1).data()(i) = 9-i; }
    testBaseCRTP.sub(1).subResize(3); 
    testBaseCRTP.sub(0).subResize(2);
    for(int i = 0; i < testBaseCRTP.sub(0).data().rows(); ++i) { testBaseCRTP.sub(0).data()(i) = 1+i; }
    cout<<endl<<"Starting baseVirtual Test"<<endl<<endl;
    testBaseVirtual.subResize(5);
    testBaseVirtual.sub(1).subResize(5); 
    for(int i = 0; i < testBaseVirtual.sub(1).data().rows(); ++i) { testBaseVirtual.sub(1).data()(i) = 9-i; }
    testBaseVirtual.sub(1).subResize(3); 
    testBaseVirtual.sub(0).subResize(2);
    for(int i = 0; i < testBaseVirtual.sub(0).data().rows(); ++i) { testBaseVirtual.sub(0).data()(i) = 1+i; }
    
    cout<<endl<<endl;
    
    
    TestDepth0 bla;
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    bla.subResize(5);
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    bla.sub(1).subResize(5); 
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    for(int i = 0; i < bla.sub(1).data().rows(); ++i) { bla.sub(1).data()(i) = 9-i; }
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    bla.sub(1).subResize(3); 
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    bla.sub(0).subResize(2);
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    for(int i = 0; i < bla.sub(0).data().rows(); ++i) { bla.sub(0).data()(i) = 1+i; }
    cout<<"dataBuffer="<<bla.data().transpose()<<endl;
    cout<<bla.data().rows()<<endl;
    cout<<bla.sub(0).data().rows()<<endl;
    cout<<bla.sub(1).data().rows()<<endl<<endl;
    
    cout<<bla.data()<<endl<<endl;
    cout<<bla.data().transpose()<<endl<<endl;
    cout<<bla.sub(0).data()<<endl<<endl;
    cout<<bla.sub(1).data()<<endl<<endl;
    
    cout<<"isAllStaticUnder="<<bla.MapSize<<endl;
    
    bla.sub(0).sub(0);
    testBaseCRTP.sub(0).sub(0);
    testBaseVirtual.sub(0).sub(0);
    
    return 0;
}
