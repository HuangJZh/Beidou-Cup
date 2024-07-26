

#ifndef FAIM_ASSERT_H_
#define FAIM_ASSERT_H_

#include <iostream>


#define UNUSED(x) (void)(x)

inline void assertion_failed(char const* expr,
							 char const* function,
                             char const* file,
							 long 		 line)
{
	std::cerr << "***** Assertion (" << expr << ") failed in " << function
			  << ":\n"
			  << file << ':' << line << ":" << std::endl;
	std::abort();
}



#define FAIM_LIKELY(x) __builtin_expect(x, 1)

//#if defined(FAIM_DISABLE_ASSERTS)

#define FAIM_ASSERT(expr) ((void)0)

#define FAIM_ASSERT_STREAM(expr, msg) ((void)0)

#else

#define FAIM_ASSERT(expr)                                               \
  (FAIM_LIKELY(!!(expr))                                                \
       ? ((void)0)                                                        \
       : ::assertion_failed(#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__))


#define FAIM_ASSERT_STREAM(expr, msg)                                    \
  (FAIM_LIKELY(!!(expr))                                                 \
       ? ((void)0)                                                         \
       : (std::cerr << msg << std::endl,                                   \
          ::assertion_failed(#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__)))

//#endif

#endif
