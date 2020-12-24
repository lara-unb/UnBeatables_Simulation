#include <qi/applicationsession.hpp>
#include <boost/shared_ptr.hpp>
#include "./vrepsimulation.h"
#include <iostream>
#include <boost/config/warning_disable.hpp>





int main(int argc, char* argv[])
{
  qi::ApplicationSession app(argc, argv);
  app.startSession();
  qi::SessionPtr session = app.session();
  session->registerService("VrepSimulation", qi::AnyObject(boost::make_shared<VrepSimulation>(session)));
  app.run();

  return 0;
}
