#include <iostream>
#include <vector>
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "rs232/rs232.h"
#include "spatial"

#define PI 3.14159265358979323
#define RADIANS_TO_DEGREES (180./PI)

an_decoder_t an_decoder;

int an_packet_transmit(an_packet_t *an_packet){
    an_packet_encode(an_packet);
    return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

void set_packet_periods(){
    an_packet_t *an_packet;
    packet_periods_packet_t packet_periods_packet;

    packet_period_t raw_sensors_period;
    packet_period_t system_state_period;

    packet_periods_packet.clear_existing_packets = TRUE;
    packet_periods_packet.permanent = FALSE;


    system_state_period.packet_id = 20;
    system_state_period.period = 50;
    raw_sensors_period.packet_id = 28;
    raw_sensors_period.period = 20; // ms

    //packet_periods_packet.packet_periods[0] = system_state_period;
    packet_periods_packet.packet_periods[0] = raw_sensors_period;

    an_packet = encode_packet_periods_packet(&packet_periods_packet);

    an_packet_transmit(an_packet);

    an_packet_free(&an_packet);
}

bool Spatial::connect(char *port, int baudrate){
    if (OpenComport(port, baudrate)){
        return false;
    }
    an_decoder_initialise(&an_decoder);

    set_packet_periods();

    return true;
}

bool Spatial::poll(){
    system_state_packet_t system_state_packet;
    raw_sensors_packet_t raw_sensors_packet;
    acknowledge_packet_t acknowledge_packet;

    bool ret = false;
    int bytes_received;

    
    an_packet_t *an_packet;

    if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0){
        std::cout << "Bytes received: " << bytes_received << "\n";
        /* increment the decode buffer length by the number of bytes received */
        an_decoder_increment(&an_decoder, bytes_received);

        /* decode all the packets in the buffer */
        buffer = {}; // (return buffer)
        while ((an_packet = an_packet_decode(&an_decoder)) != NULL){
            if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */{
                if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0){
                    
                    IMU_acc current = {raw_sensors_packet.accelerometers[0],
                                        raw_sensors_packet.accelerometers[1],
                                        raw_sensors_packet.accelerometers[2]};
                    buffer.push_back(current);
                    ret = true;
                }
            }else if (an_packet->id == packet_id_acknowledge) /* acknowledgement packet */{
                if(decode_acknowledge_packet(&acknowledge_packet, an_packet) == 0){
                    std::cout << "Acknowledge";
                    std::cout << " ID: " << (int) acknowledge_packet.packet_id << "\n";
                    switch(acknowledge_packet.acknowledge_result){
                        case acknowledge_success:
                            std::cout << "Acknowledge success";
                            break;
                        case acknowledge_failure_crc:
                            std::cout << "Acknowledge failure crc";
                            break;
                        case acknowledge_failure_length:
                            std::cout << "Acknowledge failure length";
                            break;
                        case acknowledge_failure_range:
                            std::cout << "Acknowledge failure range";
                            break;
                        case acknowledge_failure_flash:
                            std::cout << "Acknowledge failure flash";
                            break;
                        case acknowledge_failure_not_ready:
                            std::cout << "Acknowledge failure not ready";
                            break;
                        case acknowledge_failure_unknown_packet:
                            std::cout << "Acknowledge failure unknown packet";
                            break;
                        default:
                            std::cout << "Acknowledge Error\n";
                            break;
                    }
                    std::cout << "\n";
                }
            }else{
                std::cout << "Packet ID " << (int) an_packet->id;
                std::cout << " of Length " << (int) an_packet->length << "\n";
            }
            /* Ensure that you free the an_packet when your done with it or you will leak memory */
            an_packet_free(&an_packet);
        }
    }
    return ret;
}
