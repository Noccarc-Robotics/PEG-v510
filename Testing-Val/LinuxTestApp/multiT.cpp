
#include <iostream>
#include <thread>
using namespace std;
  
// A dummy function
void foo(int Z)
{
	while(1) {
	
		cout << "Hello From: ID = " << Z << endl;
		std::this_thread::yield();
	
	}

}
  
int main()
{
    cout << "Threads 1 and 2 and 3 "
         "operating independently" << endl;
         
    	    thread th1(foo, 1);
	    thread th2(foo, 2);
	    thread th3(foo, 3);     
    
            // This thread is launched by using 
	    // function pointer as callable
 	while(1) {	  
	    // Wait for the threads to finish
	    // Wait for thread t1 to finish
	    th1.join();
	  
	    // Wait for thread t2 to finish
	    th2.join();
	  
	    // Wait for thread t3 to finish
	    th3.join();
	 }
  
    return 0;
}
