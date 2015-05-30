#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>
#include <stdlib.h>

int main ( int argc, char** argv )
{
  try
    {

      ClientSocket client_socket ( argv[1], 30000 );

      std::string reply, s;

      std::cout << "Write a message to be sent to the clinet: " << std:: endl;
      while(true) {
      try
	{
      std::cin >> s;
	  client_socket << s;
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

  return 0;
}
