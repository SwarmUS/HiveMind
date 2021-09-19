#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "IBittyBuzzBytecode.h"
#include "IBittyBuzzClosureRegister.h"
#include "IBittyBuzzMessageHandler.h"
#include "IBittyBuzzMessageService.h"
#include "IBittyBuzzNeighborsManager.h"
#include "IBittyBuzzStringResolver.h"
#include "IBittyBuzzVm.h"
#include <application-interface/IUserUI.h>
#include <array>
#include <bsp/IBSP.h>
#include <bsp/IUserInterface.h>
#include <logger/ILogger.h>

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    /**
     *@brief The constructor of the bbvm
     *@param bytecode the bytecode that the vm will run
     *@param stringResolver the string resolver used in the VM
     *@param messageHandler a reference to a buzz message handler, called on step
     *@param closureRegister a reference to the closure register
     *@param messageService a reference to the message service
     *@param neighborsManager a reference to a neighbors manager
     *@param userUI the ui accessible to the user (led, hex display, etc)
     *@param bsp a reference to the bsp
     *@param logger a reference to a logger
     *@param ui a reference to a user interface */
    BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                const IBittyBuzzStringResolver& stringResolver,
                IBittyBuzzMessageHandler& messageHandler,
                IBittyBuzzClosureRegister& closureRegister,
                IBittyBuzzMessageService& messageService,
                IBittyBuzzNeighborsManager& neighborsManager,
                IUserUI& userUI,
                IBSP& bsp,
                ILogger& logger,
                IUserInterface& ui);

    ~BittyBuzzVm() override = default;

    bool init(const std::reference_wrapper<IBittyBuzzLib>* bbzLibs,
              uint32_t bbzLibsLength) override;

    void terminate() override;

    BBVMRet step() override;

    bbzvm_state getState() const override;

    bbzvm_error getError() const override;

    bbzvm_instr getInstruction() const override;

  private:
    const IBittyBuzzBytecode& m_bytecode;
    const IBSP& m_bsp;
    IBittyBuzzMessageHandler& m_messageHandler;
    IBittyBuzzMessageService& m_messageService;
    IBittyBuzzNeighborsManager& m_neighborsManager;
    IBittyBuzzClosureRegister& m_closureRegister;
    ILogger& m_logger;
    IUserInterface& m_ui;

    bbzvm_t m_bbzVm;
};

#endif // __BITTYBUZZVM_H_
