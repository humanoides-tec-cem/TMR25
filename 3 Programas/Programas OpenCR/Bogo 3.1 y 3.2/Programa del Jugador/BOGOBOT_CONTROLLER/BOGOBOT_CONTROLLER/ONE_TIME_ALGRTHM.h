#ifndef ONE_TIME_ALGRTHM
#define ONE_TIME_ALGRTHM

#include "B3_POS.h"
#include "B3_ROBOT_MOVE.h"

//  ------------------------------- P O S E S       P A R A R       B O C A         A R R I B A---------------------------------------------------------------
//                                        0001  0002  0003  0004  0005  0006  0007  0008  0009  0010  0011  0012  0013  0014  0015  0016  0017  0018
double pose_Acostado[20]               = {2057, 2016, 1023, 3089, 2058, 2017, 1816, 1816, 2150, 1946, 2034, 2036, 2026, 2086, 1759, 2359, 2077, 2065, 2, 10};
double pose_Rodillas_Arriba[20]        = {2007, 2088, 1959, 2129, 4012,   77, 1784, 1784, 2150, 1946, 3404,  653,  785, 3342, 2133, 2054, 2075, 2085, 2, 10};
double pose_Baja_Rodillas[20]          = {1000, 3097, 1932, 2141, 3962,  117, 1779, 1779, 2150, 1946, 2544, 1526,  768, 3329, 2361, 1769, 2042, 1997, 1, 10};
double pose_Sube_Torso[20]             = {1160, 2907,  937, 3074, 2015, 2086, 1788, 1832, 2150, 1946, 2276, 1843, 584, 3478, 1484, 2617, 2079, 1970, 1.5, 10};
double pose_sube_pelvis[20]            = {1351, 2753, 1008, 3082, 2016, 2085, 1756, 1756, 2150, 1946, 2403, 1699,  527, 3549, 1463, 2661, 1964, 1972, 2, 10};
double pose_se_sienta[20]              = {2000, 2125, 1176, 3052, 2348, 1661, 1845, 1867, 2150, 1946, 2471, 1616, 467, 3596, 963, 3126, 2134, 1987, 2, 10};

void Stand_Up_FaceUp(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]){
  //  ------------------------------- P O S E S       P A R A R       B O C A         A R R I B A---------------------------------------------------------------
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Acostado);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Rodillas_Arriba);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Baja_Rodillas);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_Sube_Torso);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_se_sienta);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

}



//                                     <---RA----LA----RA----LA----RA----LA-||-RL----LL----RL----LL---RL-----LL---RL----LL-----RR----LL----RL
//  ------------------------------- P O S E S       P A R A R       B O C A         A B A J O ---------------------------------------------------------------
//                                        0001  0002  0003  0004  0005  0006  0007  0008  0009  0010  0011  0012  0013  0014  0015  0016  0017  0018
double pose_boca_abajo_0[20]            ={2013, 2082, 1081, 2960, 2242, 1837, 1841, 1841, 2059, 2010, 2090, 1980, 1880, 2186,  934, 3178, 2122, 1997, 2,10};   
double pose_boca_abajo_1[20]            ={2076, 2015, 2898, 1178, 3976,  139, 1842, 1842, 2059, 2010, 2090, 1977, 1887, 2184,  935, 3176, 2120, 2004, 1,10};   
double pose_boca_abajo_2[20]            ={3006, 1030, 3020, 1107, 3958,  161, 1841, 1841, 2059, 2010, 2090, 1976, 1887, 2184,  934, 3176, 2118, 2004, 1,10};   
double pose_boca_abajo_3[20]            ={3146, 1009, 2151, 2031, 4021,   68, 1846, 1846, 2059, 2010, 2090, 1978, 1890, 2185,  934, 3177, 2118, 2004, 1,10};
double pose_boca_abajo_4[20]            ={3100,  989, 1585, 2587, 3242,  863, 1830, 1830, 2059, 2010, 2460, 1579, 1339, 2762,  929, 3191, 2102, 2047, 1,10};   
double pose_boca_abajo_5[20]            ={3087,  991, 1100, 2960, 2258, 1759, 1812, 1812, 2059, 2010, 2710, 1336,  943, 3152,  922, 3188, 2057, 2055, 1,10}; 
double pose_boca_abajo_6[20]            ={2778,  952, 1430, 2983, 3507, 1852, 1785, 1785, 2059, 2010, 2718, 1332,  945, 3158,  923, 3186, 2047, 2056, 1,10}; 
double pose_boca_abajo_7[20]            ={2812,  989, 1172, 2927, 2571, 1811, 1759, 1759, 2059, 2010, 2735, 1325,  917, 3180,  930, 3183, 2026, 2063, 1,10};   
double pose_boca_abajo_8[20]            ={2841, 1374, 1276, 2718, 2676,  563, 1799, 1799, 2059, 2010, 2791, 1245,  860, 3251,  922, 3183, 2025, 2068, 1,10};  
double pose_boca_abajo_9[20]            ={2857, 1300, 1235, 2971, 2583, 1617, 1818, 1818, 2059, 2010, 2863, 1186,  750, 3332,  920, 3161, 2026, 2098, 1,10};  
double pose_boca_abajo_10[20]           ={2724, 1338, 1106, 2988, 2209, 1847, 1836, 1836, 2059, 2010, 2924, 1103,  588, 3509,  917, 3157, 2028, 2146, 2,10};  
double pose_boca_abajo_11[20]           ={2706, 1426, 1088, 2969, 2352, 1660, 1840, 1840, 2059, 2010, 3073,  981,  406, 3675,  935, 3150, 2028, 1996, 2,10};  
double pose_boca_abajo_12[20]           ={2082, 1939, 1176, 2820, 2348, 1661, 1840, 1840, 2059, 2010, 2368, 1645,  498, 3574,  958, 3069, 1959, 2040, 2,10};  
double pose_boca_abajo_13[20]           ={2082, 1939, 1176, 2820, 2348, 1661, 1840, 1840, 2150, 1946, 2387, 1641, 502, 3607, 1137, 2973, 1985, 1978, 2,10};
double pose_boca_abajo_14[20]           ={2000, 2125, 1176, 3052, 2348, 1661, 1791, 1875, 2109, 1893, 2387, 1278, 462, 3607, 1137, 2973, 1985, 1978, 3,10};  
double pose_boca_abajo_15[20]           ={2000, 2125, 1176, 3052, 2351, 1908, 1791, 1875, 2109, 1893, 2821, 1278, 462, 3594, 1340, 2776, 2078, 1875, 4,10};  

void Stand_Up_FaceDown(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]){
  //--------------------------------- P O S E S       P A R A R         B O C A       A B A J O------------------------------------------------------------
  // for(int p=0; p < siz_pos_abajo-1 ;p++){
  //   moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,poses_boca_abajo[p]);
  // }
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_0);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_1);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_2);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_3);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_4);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_5);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_6);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_7);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_8);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_9);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_10);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_11);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_12);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_13);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_boca_abajo_15);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);
}

//  ------------------------------- P O S E S       P A R A R       P A T E A R      P E L O T A      C / D E R  E C H A--------------------------------------------------------
//                                        0001  0002  0003  0004  0005  0006  0007  0008  0009  0010  0011  0012  0013  0014  0015  0016  0017  0018
//                                     <---RA----LA----RA----LA----RA----LA-||-RL----LL----RL----LL---RL-----LL---RL----LL-----RR----LL----RL
double pose_patea_1[20]                 ={2053, 2066, 997, 3055, 2055, 2045, 1801, 1830, 2322, 2197, 2275, 1653, 1552, 2741, 1757, 2378, 2367, 2208, 3,10};
double pose_patea_2[20]                 ={2053, 2066, 997, 3055, 2055, 2045, 1801, 1830, 2219, 2181, 2380, 1660, 1237, 2764, 1512, 2247, 2205, 2161, 3,10};
double pose_patea_3[20]                 ={2021, 2061, 1421, 2651, 2096, 2012, 1762, 1772, 2438, 2238, 2596, 1718,  594, 2915, 1357, 2562, 2194, 2294, 4,10};  
double pose_patea_4[20]                 ={2021, 2061, 1421, 2651, 2096, 2012, 1734, 1772, 2415, 2238, 3031, 1718, 686, 2900, 1851, 2549, 1986, 2290, 1,10};  
double pose_patea_5[20]                 ={2021, 2061, 1421, 2651, 2096, 2012, 1762, 1772, 2438, 2238, 2596, 1718,  594, 2915, 1357, 2559, 2207, 2269, 1,10}; //es la 2 otra vez pero mas rapido


void kick_der(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
                   double IDs[18]){
  //--------------------------------- P O S E S       P A R A R         P A T E A R     C O N     D E R E C H A------------------------------------------------------------
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_1);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_2);  
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_3);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_4);
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,pose_patea_5);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);
}

#endif
