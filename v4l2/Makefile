PROG=	fthd-v4l2d
MAN=
WARNS?=	6
CFLAGS+=	-I/usr/local/include
# Helpers run with mtx held, which the analysis can't follow
NO_WTHREAD_SAFETY=
LDADD=	-lcuse -lpthread

.include <bsd.prog.mk>
