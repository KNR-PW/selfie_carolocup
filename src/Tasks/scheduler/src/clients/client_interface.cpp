/**
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#include <scheduler/clients/client_interface.h>

ClientInterface::~ClientInterface()
{
  std::cout << "Pure virtual destructor is called";
}
