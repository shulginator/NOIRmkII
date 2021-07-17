const uint16_t sine[1000] = {
  0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
  1, 1, 1, 2, 2, 2, 3, 3, 3, 4,
  4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
  9, 9, 10, 11, 11, 12, 13, 13, 14, 15,
  16, 16, 17, 18, 19, 20, 21, 22, 23, 24,
  24, 25, 26, 27, 29, 30, 31, 32, 33, 34,
  35, 36, 37, 39, 40, 41, 42, 44, 45, 46,
  48, 49, 50, 52, 53, 54, 56, 57, 59, 60,
  62, 63, 65, 66, 68, 70, 71, 73, 75, 76,
  78, 80, 81, 83, 85, 86, 88, 90, 92, 94,
  95, 97, 99, 101, 103, 105, 107, 109, 111, 113,
  115, 117, 119, 121, 123, 125, 127, 129, 131, 133,
  136, 138, 140, 142, 144, 146, 149, 151, 153, 155,
  158, 160, 162, 165, 167, 169, 172, 174, 176, 179,
  181, 184, 186, 189, 191, 194, 196, 199, 201, 204,
  206, 209, 211, 214, 216, 219, 222, 224, 227, 229,
  232, 235, 237, 240, 243, 245, 248, 251, 254, 256,
  259, 262, 265, 267, 270, 273, 276, 279, 281, 284,
  287, 290, 293, 296, 299, 301, 304, 307, 310, 313,
  316, 319, 322, 325, 328, 331, 334, 337, 340, 343,
  345, 348, 351, 354, 357, 361, 364, 367, 370, 373,
  376, 379, 382, 385, 388, 391, 394, 397, 400, 403,
  406, 409, 412, 416, 419, 422, 425, 428, 431, 434,
  437, 440, 444, 447, 450, 453, 456, 459, 462, 465,
  469, 472, 475, 478, 481, 484, 487, 491, 494, 497,
  500, 503, 506, 509, 513, 516, 519, 522, 525, 528,
  531, 535, 538, 541, 544, 547, 550, 553, 556, 560,
  563, 566, 569, 572, 575, 578, 581, 584, 588, 591,
  594, 597, 600, 603, 606, 609, 612, 615, 618, 621,
  624, 627, 630, 633, 636, 639, 643, 646, 649, 652,
  655, 657, 660, 663, 666, 669, 672, 675, 678, 681,
  684, 687, 690, 693, 696, 699, 701, 704, 707, 710,
  713, 716, 719, 721, 724, 727, 730, 733, 735, 738,
  741, 744, 746, 749, 752, 755, 757, 760, 763, 765,
  768, 771, 773, 776, 778, 781, 784, 786, 789, 791,
  794, 796, 799, 801, 804, 806, 809, 811, 814, 816,
  819, 821, 824, 826, 828, 831, 833, 835, 838, 840,
  842, 845, 847, 849, 851, 854, 856, 858, 860, 862,
  864, 867, 869, 871, 873, 875, 877, 879, 881, 883,
  885, 887, 889, 891, 893, 895, 897, 899, 901, 903,
  905, 906, 908, 910, 912, 914, 915, 917, 919, 920,
  922, 924, 925, 927, 929, 930, 932, 934, 935, 937,
  938, 940, 941, 943, 944, 946, 947, 948, 950, 951,
  952, 954, 955, 956, 958, 959, 960, 961, 963, 964,
  965, 966, 967, 968, 969, 970, 971, 973, 974, 975,
  976, 976, 977, 978, 979, 980, 981, 982, 983, 984,
  984, 985, 986, 987, 987, 988, 989, 989, 990, 991,
  991, 992, 992, 993, 993, 994, 994, 995, 995, 996,
  996, 996, 997, 997, 997, 998, 998, 998, 999, 999,
  999, 999, 999, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 999, 999,
  999, 999, 999, 998, 998, 998, 997, 997, 997, 996,
  996, 996, 995, 995, 994, 994, 993, 993, 992, 992,
  991, 991, 990, 989, 989, 988, 987, 987, 986, 985,
  984, 984, 983, 982, 981, 980, 979, 978, 977, 976,
  976, 975, 974, 973, 971, 970, 969, 968, 967, 966,
  965, 964, 963, 961, 960, 959, 958, 956, 955, 954,
  952, 951, 950, 948, 947, 946, 944, 943, 941, 940,
  938, 937, 935, 934, 932, 930, 929, 927, 925, 924,
  922, 920, 919, 917, 915, 914, 912, 910, 908, 906,
  905, 903, 901, 899, 897, 895, 893, 891, 889, 887,
  885, 883, 881, 879, 877, 875, 873, 871, 869, 867,
  864, 862, 860, 858, 856, 854, 851, 849, 847, 845,
  842, 840, 838, 835, 833, 831, 828, 826, 824, 821,
  819, 816, 814, 811, 809, 806, 804, 801, 799, 796,
  794, 791, 789, 786, 784, 781, 778, 776, 773, 771,
  768, 765, 763, 760, 757, 755, 752, 749, 746, 744,
  741, 738, 735, 733, 730, 727, 724, 721, 719, 716,
  713, 710, 707, 704, 701, 699, 696, 693, 690, 687,
  684, 681, 678, 675, 672, 669, 666, 663, 660, 657,
  655, 652, 649, 646, 643, 639, 636, 633, 630, 627,
  624, 621, 618, 615, 612, 609, 606, 603, 600, 597,
  594, 591, 588, 584, 581, 578, 575, 572, 569, 566,
  563, 560, 556, 553, 550, 547, 544, 541, 538, 535,
  531, 528, 525, 522, 519, 516, 513, 509, 506, 503,
  500, 497, 494, 491, 487, 484, 481, 478, 475, 472,
  469, 465, 462, 459, 456, 453, 450, 447, 444, 440,
  437, 434, 431, 428, 425, 422, 419, 416, 412, 409,
  406, 403, 400, 397, 394, 391, 388, 385, 382, 379,
  376, 373, 370, 367, 364, 361, 357, 354, 351, 348,
  345, 343, 340, 337, 334, 331, 328, 325, 322, 319,
  316, 313, 310, 307, 304, 301, 299, 296, 293, 290,
  287, 284, 281, 279, 276, 273, 270, 267, 265, 262,
  259, 256, 254, 251, 248, 245, 243, 240, 237, 235,
  232, 229, 227, 224, 222, 219, 216, 214, 211, 209,
  206, 204, 201, 199, 196, 194, 191, 189, 186, 184,
  181, 179, 176, 174, 172, 169, 167, 165, 162, 160,
  158, 155, 153, 151, 149, 146, 144, 142, 140, 138,
  136, 133, 131, 129, 127, 125, 123, 121, 119, 117,
  115, 113, 111, 109, 107, 105, 103, 101, 99, 97,
  95, 94, 92, 90, 88, 86, 85, 83, 81, 80,
  78, 76, 75, 73, 71, 70, 68, 66, 65, 63,
  62, 60, 59, 57, 56, 54, 53, 52, 50, 49,
  48, 46, 45, 44, 42, 41, 40, 39, 37, 36,
  35, 34, 33, 32, 31, 30, 29, 27, 26, 25,
  24, 24, 23, 22, 21, 20, 19, 18, 17, 16,
  16, 15, 14, 13, 13, 12, 11, 11, 10, 9,
  9, 8, 8, 7, 7, 6, 6, 5, 5, 4,
  4, 4, 3, 3, 3, 2, 2, 2, 1, 1,
  1, 1, 1, 0, 0, 0, 0, 0, 0, 0
  };