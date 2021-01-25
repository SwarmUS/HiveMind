#include "BSPUtils.h"
#include <csignal>

void BSPUtils::blockSignals() {
    sigset_t mask;
    sigfillset(&mask);
    sigprocmask(SIG_SETMASK, &mask, nullptr);
}

void BSPUtils::unblockSignals() {
    sigset_t mask;
    sigemptyset(&mask);
    sigprocmask(SIG_SETMASK, &mask, nullptr);
}