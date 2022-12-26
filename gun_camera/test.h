#ifndef _DEVASTAR_TEST_H_
#define _DEVASTAR_TEST_H_

#define CHECK_EQUAL(VALUE, EXPECTED, NAME) \
  if (VALUE != EXPECTED) \
    { \
      std::cerr  << "\033[1;31mFAILED\033[0m " << NAME << " expected: " << EXPECTED << " value: " << VALUE <<"\n"; \
    } \
  else \
    { \
      std::cerr  << "\033[1;32mPASSED\033[0m " << NAME << " expected: " << EXPECTED << " value: " << VALUE <<"\n"; \
    } 


#define CHECK_EQUAL_REAL(VALUE, EXPECTED, NAME, MAX_DIFF ) \
  if (abs( VALUE -EXPECTED) > MAX_DIFF ) \
    { \
      std::cerr  << "\033[1;31mFAILED\033[0m " << NAME << " expected: " << EXPECTED << " value: " << VALUE <<"\n"; \
    } \
  else \
    { \
      std::cerr  << "\033[1;32mPASSED\033[0m " << NAME << " expected: " << EXPECTED << " value: " << VALUE <<"\n"; \
    } 


#endif // _DEVASTAR_TEST_H_



