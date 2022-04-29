#include <iostream>
#include <zmq_addon.hpp>

int main()
{
    zmq::context_t ctx(1);
    zmq::socket_t subscriber(ctx, zmq::socket_type::sub);

    subscriber.connect("tcp://localhost:5556");

    //  Thread3 opens ALL envelopes
    subscriber.set(zmq::sockopt::subscribe, "");

    std::string stop_trigger("Goodbye");

    std::cout << "[CPP]: Listening..." << std::endl;
    while (1) {
        // Receive all parts of the message
        std::vector<zmq::message_t> recv_msgs;
        zmq::recv_result_t result =
          zmq::recv_multipart(subscriber, std::back_inserter(recv_msgs));
        assert(result && "recv failed");

        std::cout << "[CPP] Received: " << recv_msgs[0].to_string() << std::endl;

        if (stop_trigger.compare(recv_msgs[0].to_string())  == 0)
            break;
    }

    std::cout << "[CPP]: Done." << std::endl;

    return 0;
}
