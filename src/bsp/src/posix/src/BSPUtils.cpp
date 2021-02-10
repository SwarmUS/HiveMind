#include "BSPUtils.h"

sigset_t BSPUtils::blockSignals() {
    sigset_t mask;
    sigset_t omask;

    sigfillset(&mask);
    pthread_sigmask(SIG_SETMASK, &mask, &omask);

    return omask;
}

void BSPUtils::unblockSignals(sigset_t omask) { pthread_sigmask(SIG_SETMASK, &omask, nullptr); }