#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>
#include <limits>

ignition::transport::Node node;
ignition::transport::Node::Publisher pub;

//////////////////////////////////////////////////
/// \brief Callback para lidar com dados do LIDAR e publicar a menor distância.
void cb(const ignition::msgs::LaserScan &_msg)
{
  double minDistance = std::numeric_limits<double>::infinity();

  for (int i = 0; i < _msg.ranges_size(); ++i)
  {
    double range = _msg.ranges(i);
    if (range < minDistance)
      minDistance = range;
  }

  ignition::msgs::Double distanceMsg;
  distanceMsg.set_data(minDistance);
  pub.Publish(distanceMsg);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::string topic_sub = "/lidar";           // Tópico de entrada (dados do LIDAR)
  std::string topic_pub = "/scan";  // Tópico de saída (menor distância)

  // Criar publisher
  pub = node.Advertise<ignition::msgs::Double>(topic_pub);

  // Inscrever no tópico do LIDAR
  if (!node.Subscribe(topic_sub, cb))
  {
    std::cerr << "Erro ao se inscrever no tópico [" << topic_sub << "]" << std::endl;
    return -1;
  }

  // Mantém o processo ativo
  ignition::transport::waitForShutdown();

  return 0;
}
