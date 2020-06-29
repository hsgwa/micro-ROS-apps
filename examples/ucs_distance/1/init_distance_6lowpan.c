#include <stdio.h>
#include <string.h>
#include <stdlib.h>


// #include <nuttx/config.h>
// #include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
// #include <nuttx/fs/ioctl.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>       ///
#include "wireless/ieee802154.h"                            ////

// #include "i8sak.h"


#include "init_distance_6lowpan.h"

union i8sak_set_u
{
  union ieee802154_attr_u attr;
};

// int init_6lowpan(FAR struct i8sak_s *i8sak)
// {
//     char buffer[10];
//     union i8sak_set_u u;
//     int fd = 0;

//     // if (strcmp(argv[argind], "chan") == 0)
//     sprintf(buffer,"%d",DISTANCE_CHANNEL);                  //Set the radio channel.   
//     u.attr.phy.chan = i8sak_str2luint8(buffer);
//     // u.attr.phy.chan = i8sak_str2luint8(argv[argind + 1]);
//     ieee802154_setchan(fd, u.attr.phy.chan);
//     i8sak->chan = u.attr.phy.chan;

//     return 0;
// }

int init_6lowpan()
{
    char buffer[10];
    union i8sak_set_u u;
    int fd = 0;
    uint8_t ret;
    bool b_ret;
        
    fd = open("/dev/ieee0", O_RDWR);
    if(fd < 0)
    {
        printf(" Error fd < 0 \n");
        return 0;
    }
    ieee802154_getchan(fd, &ret);
    printf(" chan %d \n", ret);

    ieee802154_getpanid(fd, &ret);
    printf(" panid %d \n", ret);

    // ieee802154_getsaddr(fd, &ret);
    // printf(" sadr %d \n", ret);

    // ieee802154_geteaddr(fd, &ret);
    // printf(" eadr %d \n", ret);

    // ieee802154_getcoordsaddr(fd, &ret);
    // printf(" c_sadr %d \n", ret);

    // ieee802154_getcoordeaddr(fd, &ret);
    // printf(" c_eadr %d \n", ret);

    // ieee802154_getpromisc(fd, &b_ret);
    // printf(" promisc %d \n", b_ret);

    // ieee802154_getrxonidle(fd, &b_ret);
    // printf(" rxonidle %d \n", b_ret);

    close(fd);
    return 0;
}

int init_distance_6lowpan(void)
{
    // //6lowpan configuration process
    char buffer[256]; 
    system("ifdown wpan0"); // Is necessary to bring down the network to configure.

    // system("i8sak wpan0 startpan cd:ab"); //Set the radio as an endpoint.
    sprintf(buffer,"i8sak wpan0 startpan %02x:%02x", DISTANCE_PAN_ID & 0xff, DISTANCE_PAN_ID >> 8); //Set the radio as an endpoint.  
    system(buffer);
printf(" %s \n", buffer);
    // system("i8sak set chan 11"); //Set the radio channel.
    sprintf(buffer,"i8sak set chan %d",DISTANCE_CHANNEL); //Set the radio channel.   
    system(buffer);
printf(" %s \n", buffer);

    // system("i8sak set panid cd:ab"); //Set network PAN ID.
    sprintf(buffer,"i8sak set panid %02x:%02x", DISTANCE_PAN_ID & 0xff, DISTANCE_PAN_ID  >> 8); //Set network PAN ID
    system(buffer);
printf(" %s \n", buffer);


    // sprintf(buffer,"i8sak set saddr 42:%02x",CONFIG_UCS_DISTANCE_EXAMPLE_ID); // Set the short address of the radio
    sprintf(buffer,"i8sak set saddr 42:%02x",DISTANCE_DEVICE_ID); // Set the short address of the radio
    system(buffer);
printf(" %s \n", buffer);

    // sprintf(buffer, "i8sak set eaddr 00:fa:de:00:de:ad:be:%02x", CONFIG_UCS_DISTANCE_EXAMPLE_ID); // TODO: This won't work on the lastest version of NuttX
    sprintf(buffer, "i8sak set eaddr 00:fa:de:00:de:ad:be:%02x", DISTANCE_DEVICE_ID); // TODO: This won't work on the lastest version of NuttX
    system(buffer);
printf(" %s \n", buffer);

    system("i8sak acceptassoc");
    system("ifup wpan0"); // Bring up the network.

    system("mount -t procfs /proc");// Mount the proc file system to check the connection data.
    printf("Connection data\r\n");
    system("cat proc/net/wpan0");

    return 0;
}