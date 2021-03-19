#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "IBittyBuzzBytecode.h"
#include "IBittyBuzzClosureRegister.h"
#include "IBittyBuzzMessageHandler.h"
#include "IBittyBuzzMessageService.h"
#include "IBittyBuzzStringResolver.h"
#include "IBittyBuzzVm.h"
#include <array>
#include <bsp/IBSP.h>
#include <bsp/IUserInterface.h>
#include <logger/ILogger.h>

#define BBZ_MSG_BUFF_SIZE 16

class UserFunctionRegister {
  public:
    UserFunctionRegister(uint8_t strId, bbzvm_funp functionPtr);
    uint8_t m_strId;
    bbzvm_funp m_functionPtr;

  private:
    UserFunctionRegister();
};

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    /**
     *@brief The constructor of the bbvm
     *@param bytecode the bytecode that the vm will run
     *@param stringResolver the string resolver used in the VM
     *@param messageHandler a reference to a buzz message handler, called on step
     *@param closureRegister a reference to the closure register
     *@param messageService a reference to the message service
     *@param bsp a reference to the bsp
     *@param logger a reference to a logger
     *@param ui a reference to a user interface
     *@param container the provided iterator
     *@tparam Container an iterator of any sort (stl container) that returns a FunctionRegister*/
    template <typename Container>
    BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                const IBittyBuzzStringResolver& stringResolver,
                IBittyBuzzMessageHandler& messageHandler,
                IBittyBuzzClosureRegister& closureRegister,
                IBittyBuzzMessageService& messageService,
                const IBSP& bsp,
                ILogger& logger,
                IUserInterface& ui,
                const Container& container);

    ~BittyBuzzVm() override = default;

    bool step() override;

    bbzvm_state getSate() const override;

    bbzvm_error getError() const override;

  private:
    const IBittyBuzzBytecode& m_bytecode;
    const IBSP& m_bsp;
    IBittyBuzzMessageHandler& m_messageHandler;
    IBittyBuzzMessageService& m_messageService;
    ILogger& m_logger;
    IUserInterface& m_ui;

    bbzvm_t m_bbzVm;
    std::array<uint8_t, BBZ_MSG_BUFF_SIZE> m_bbzMsgBuff;
    bbzmsg_payload_t m_bbzPayloadBuff;
};

#include "BittyBuzzVm.tpp"

#endif // __BITTYBUZZVM_H_
