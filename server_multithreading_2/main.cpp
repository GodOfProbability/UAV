#include <iostream>
#include "ClientSocket.h"
#include "SocketException.h"
#include <string>
#include <stdlib.h>
#include "ServerSocket.h"
#include <fstream>
extern "C"
 {
    #include <pthread.h>
    #include <unistd.h>
 }



using namespace std;
const int NUMBER_OF_THREADS = 5;
void* odroid1(void* arg1);
void* odroid2(void* arg2);

int main()
{    
     pthread_t t1,t2 ;

     cout << "Starting all threads..." << endl;
     string msg1 ("inside odroid1");
     string msg2 ("inside odroid2");

     int result1 = pthread_create(&t1, NULL, odroid1, reinterpret_cast<void*>(&msg1));
     cout<<"the value of result1:"<<result1<<endl;
     if (result1)
     {
         cout << "Error creating thread " << result1 << ". Return code:" << result1 <<  endl;
     }
    sleep(5);
     int result2 = pthread_create(&t2, NULL, odroid2, reinterpret_cast<void*>(&msg2));

     if (result2 !=0)
     {
         cout << "Error creating thread " << result2 << ". Return code:" << result2 <<  endl;
     }

     pthread_exit(NULL);
     return 0;

}


//this is th thread for odroid1 which is acting as client for the host station.
 void* odroid1(void* arg1)
 {
      cout << *(reinterpret_cast<string*>(arg1)) << endl ;
      std::cout << "running here....\n";
      ofstream outfile, outfil_temp;
      std::string data;
      int i=0;
      try
        {
          // Create the socket
          ServerSocket server ( 30000 );
          //std::cout<<"I am in the outer loop";
          while ( true )
            {

              ServerSocket new_sock;
              server.accept ( new_sock );
                  //std::cout <<"I am in the inner loop";

              try
                {
                  while ( true )
                    { 
                      //outfile.open("data_od1.dat",ios::out|ios::trunc);
                      //outfile.close();
                      outfile.open("data_od1.dat",ios::out|ios::app);
                      i++;
                      //cin.ignore('\n');
                      new_sock >> data;
                      new_sock << data;
                      
                      outfile<<data;
                      outfile.flush();
                      if (outfile.is_open())
                        cout<<"atleast file is open";
                      outfile.close();
                      std::cout << "Clinet sent and also read into the file: "<<i<<" " << data << std::endl;
                      //cin.ignore(1,'\n');
                      outfil_temp.open("data_od1.dat",ios::out|ios::trunc);
                      sleep(1);
                      outfil_temp.close();
                      
                    }
                }
              catch ( SocketException& ) {throw;}

            }
        }
      catch ( SocketException& e )
        {
          std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
        }
       
    pthread_exit(NULL);

    return 0;

 }




 void* odroid2(void* arg2)
 {
    ifstream infile;
    
    long int p=0;
    try
    {

      ClientSocket client_socket ( "192.168.0.23", 30000 );

      std::string reply, s;
      // this loop should be controlles by the gost station and 
      // should only be run when you reciecve anything from odroid1 and you want to give command to odroid 2.
      std::cout << "Write a message to be sent to the client: " << std:: endl;
      while(true)
      {
     try
	    {
            sleep(1);
            infile.open("data_od1.dat",ios::in);
            //std::cin >> s;
            infile.seekg(p,ios::beg);

            getline(infile,s);

  	        client_socket << s;
            
            //std::cout<<"read from file"<<s;
  	        infile.close();
  	        //client_socket >> reply;
	    }
     catch ( SocketException& ) {}

      //std::cout << "We received this response from the server:\n\"" << reply << "\"\n";;
    }
    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }
     
     pthread_exit(NULL);

 }
