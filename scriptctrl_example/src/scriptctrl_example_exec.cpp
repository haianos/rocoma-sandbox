// rocoma ros example
#include "scriptctrl_example/ScriptCtrl.hpp"

// any node
#include "any_node/Nodewrap.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<scriptctrl_example::ScriptCtrl> node(argc, argv, "scriptctrl_example", 4);
  node.execute();
  return 0;
}
