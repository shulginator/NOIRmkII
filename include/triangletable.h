const uint16_t triangle[1000] = {
  2, 4, 6, 8, 10, 12, 14, 16, 18, 20,
  22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
  42, 44, 46, 48, 50, 52, 54, 56, 58, 60,
  62, 64, 66, 68, 70, 72, 74, 76, 78, 80,
  82, 84, 86, 88, 90, 92, 94, 96, 98, 100,
  102, 104, 106, 108, 110, 112, 114, 116, 118, 120,
  122, 124, 126, 128, 130, 132, 134, 136, 138, 140,
  142, 144, 146, 148, 150, 152, 154, 156, 158, 160,
  162, 164, 166, 168, 170, 172, 174, 176, 178, 180,
  182, 184, 186, 188, 190, 192, 194, 196, 198, 200,
  202, 204, 206, 208, 210, 212, 214, 216, 218, 220,
  222, 224, 226, 228, 230, 232, 234, 236, 238, 240,
  242, 244, 246, 248, 250, 252, 254, 256, 258, 260,
  262, 264, 266, 268, 270, 272, 274, 276, 278, 280,
  282, 284, 286, 288, 290, 292, 294, 296, 298, 300,
  302, 304, 306, 308, 310, 312, 314, 316, 318, 320,
  322, 324, 326, 328, 330, 332, 334, 336, 338, 340,
  342, 344, 346, 348, 350, 352, 354, 356, 358, 360,
  362, 364, 366, 368, 370, 372, 374, 376, 378, 380,
  382, 384, 386, 388, 390, 392, 394, 396, 398, 400,
  402, 404, 406, 408, 410, 412, 414, 416, 418, 420,
  422, 424, 426, 428, 430, 432, 434, 436, 438, 440,
  442, 444, 446, 448, 450, 452, 454, 456, 458, 460,
  462, 464, 466, 468, 470, 472, 474, 476, 478, 480,
  482, 484, 486, 488, 490, 492, 494, 496, 498, 500,
  502, 504, 506, 508, 510, 512, 514, 516, 518, 520,
  522, 524, 526, 528, 530, 532, 534, 536, 538, 540,
  542, 544, 546, 548, 550, 552, 554, 556, 558, 560,
  562, 564, 566, 568, 570, 572, 574, 576, 578, 580,
  582, 584, 586, 588, 590, 592, 594, 596, 598, 600,
  602, 604, 606, 608, 610, 612, 614, 616, 618, 620,
  622, 624, 626, 628, 630, 632, 634, 636, 638, 640,
  642, 644, 646, 648, 650, 652, 654, 656, 658, 660,
  662, 664, 666, 668, 670, 672, 674, 676, 678, 680,
  682, 684, 686, 688, 690, 692, 694, 696, 698, 700,
  702, 704, 706, 708, 710, 712, 714, 716, 718, 720,
  722, 724, 726, 728, 730, 732, 734, 736, 738, 740,
  742, 744, 746, 748, 750, 752, 754, 756, 758, 760,
  762, 764, 766, 768, 770, 772, 774, 776, 778, 780,
  782, 784, 786, 788, 790, 792, 794, 796, 798, 800,
  802, 804, 806, 808, 810, 812, 814, 816, 818, 820,
  822, 824, 826, 828, 830, 832, 834, 836, 838, 840,
  842, 844, 846, 848, 850, 852, 854, 856, 858, 860,
  862, 864, 866, 868, 870, 872, 874, 876, 878, 880,
  882, 884, 886, 888, 890, 892, 894, 896, 898, 900,
  902, 904, 906, 908, 910, 912, 914, 916, 918, 920,
  922, 924, 926, 928, 930, 932, 934, 936, 938, 940,
  942, 944, 946, 948, 950, 952, 954, 956, 958, 960,
  962, 964, 966, 968, 970, 972, 974, 976, 978, 980,
  982, 984, 986, 988, 990, 992, 994, 996, 998, 1000,
  998, 996, 994, 992, 990, 988, 986, 984, 982, 980,
  978, 976, 974, 972, 970, 968, 966, 964, 962, 960,
  958, 956, 954, 952, 950, 948, 946, 944, 942, 940,
  938, 936, 934, 932, 930, 928, 926, 924, 922, 920,
  918, 916, 914, 912, 910, 908, 906, 904, 902, 900,
  898, 896, 894, 892, 890, 888, 886, 884, 882, 880,
  878, 876, 874, 872, 870, 868, 866, 864, 862, 860,
  858, 856, 854, 852, 850, 848, 846, 844, 842, 840,
  838, 836, 834, 832, 830, 828, 826, 824, 822, 820,
  818, 816, 814, 812, 810, 808, 806, 804, 802, 800,
  798, 796, 794, 792, 790, 788, 786, 784, 782, 780,
  778, 776, 774, 772, 770, 768, 766, 764, 762, 760,
  758, 756, 754, 752, 750, 748, 746, 744, 742, 740,
  738, 736, 734, 732, 730, 728, 726, 724, 722, 720,
  718, 716, 714, 712, 710, 708, 706, 704, 702, 700,
  698, 696, 694, 692, 690, 688, 686, 684, 682, 680,
  678, 676, 674, 672, 670, 668, 666, 664, 662, 660,
  658, 656, 654, 652, 650, 648, 646, 644, 642, 640,
  638, 636, 634, 632, 630, 628, 626, 624, 622, 620,
  618, 616, 614, 612, 610, 608, 606, 604, 602, 600,
  598, 596, 594, 592, 590, 588, 586, 584, 582, 580,
  578, 576, 574, 572, 570, 568, 566, 564, 562, 560,
  558, 556, 554, 552, 550, 548, 546, 544, 542, 540,
  538, 536, 534, 532, 530, 528, 526, 524, 522, 520,
  518, 516, 514, 512, 510, 508, 506, 504, 502, 500,
  498, 496, 494, 492, 490, 488, 486, 484, 482, 480,
  478, 476, 474, 472, 470, 468, 466, 464, 462, 460,
  458, 456, 454, 452, 450, 448, 446, 444, 442, 440,
  438, 436, 434, 432, 430, 428, 426, 424, 422, 420,
  418, 416, 414, 412, 410, 408, 406, 404, 402, 400,
  398, 396, 394, 392, 390, 388, 386, 384, 382, 380,
  378, 376, 374, 372, 370, 368, 366, 364, 362, 360,
  358, 356, 354, 352, 350, 348, 346, 344, 342, 340,
  338, 336, 334, 332, 330, 328, 326, 324, 322, 320,
  318, 316, 314, 312, 310, 308, 306, 304, 302, 300,
  298, 296, 294, 292, 290, 288, 286, 284, 282, 280,
  278, 276, 274, 272, 270, 268, 266, 264, 262, 260,
  258, 256, 254, 252, 250, 248, 246, 244, 242, 240,
  238, 236, 234, 232, 230, 228, 226, 224, 222, 220,
  218, 216, 214, 212, 210, 208, 206, 204, 202, 200,
  198, 196, 194, 192, 190, 188, 186, 184, 182, 180,
  178, 176, 174, 172, 170, 168, 166, 164, 162, 160,
  158, 156, 154, 152, 150, 148, 146, 144, 142, 140,
  138, 136, 134, 132, 130, 128, 126, 124, 122, 120,
  118, 116, 114, 112, 110, 108, 106, 104, 102, 100,
  98, 96, 94, 92, 90, 88, 86, 84, 82, 80,
  78, 76, 74, 72, 70, 68, 66, 64, 62, 60,
  58, 56, 54, 52, 50, 48, 46, 44, 42, 40,
  38, 36, 34, 32, 30, 28, 26, 24, 22, 20,
  18, 16, 14, 12, 10, 8, 6, 4, 2, 0
};