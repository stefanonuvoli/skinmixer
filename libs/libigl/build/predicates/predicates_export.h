
#ifndef PREDICATES_EXPORT_H
#define PREDICATES_EXPORT_H

#ifdef PREDICATES_STATIC_DEFINE
#  define PREDICATES_EXPORT
#  define PREDICATES_NO_EXPORT
#else
#  ifndef PREDICATES_EXPORT
#    ifdef predicates_EXPORTS
        /* We are building this library */
#      define PREDICATES_EXPORT 
#    else
        /* We are using this library */
#      define PREDICATES_EXPORT 
#    endif
#  endif

#  ifndef PREDICATES_NO_EXPORT
#    define PREDICATES_NO_EXPORT 
#  endif
#endif

#ifndef PREDICATES_DEPRECATED
#  define PREDICATES_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef PREDICATES_DEPRECATED_EXPORT
#  define PREDICATES_DEPRECATED_EXPORT PREDICATES_EXPORT PREDICATES_DEPRECATED
#endif

#ifndef PREDICATES_DEPRECATED_NO_EXPORT
#  define PREDICATES_DEPRECATED_NO_EXPORT PREDICATES_NO_EXPORT PREDICATES_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef PREDICATES_NO_DEPRECATED
#    define PREDICATES_NO_DEPRECATED
#  endif
#endif

#endif /* PREDICATES_EXPORT_H */
