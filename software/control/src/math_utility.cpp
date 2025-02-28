#include "math_utility.hpp"

int sign (float val) {
    if (val > 0) {
      return 1;
    } else if (val < 0) {
      return -1;
    } else {
      return 0;
    }
}
