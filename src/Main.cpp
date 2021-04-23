#include <AbstractTask.h>
#include <Task.h>
#include <bittybuzz/BittyBuzzContainer.h>
#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzSystem.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <cstdlib>
#include <interloc/IInterloc.h>
#include <interloc/InterlocContainer.h>
#include <interloc/InterlocMessageHandler.h>
#include <logger/Logger.h>
#include <logger/LoggerContainer.h>
#include <message-handler/GreetHandler.h>
#include <message-handler/GreetSender.h>
#include <message-handler/MessageDispatcher.h>
#include <message-handler/MessageHandlerContainer.h>
#include <message-handler/MessageSender.h>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>

#include <pheromones/HiveMindHostAccumulatorSerializer.h>

constexpr uint16_t gc_taskNormalPriority = tskIDLE_PRIORITY + 1;
constexpr uint16_t gc_taskHighPriority = tskIDLE_PRIORITY + 30; // Higher priority then LwIP

// Need to return the proper comm interface
typedef std::optional<std::reference_wrapper<ICommInterface>> (*CommInterfaceGetter)();

class BittyBuzzTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    BittyBuzzTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_bytecode(BittyBuzzFactory::createBittyBuzzBytecode(m_logger)),
        m_stringResolver(BittyBuzzFactory::createBittyBuzzStringResolver(m_logger)),
        m_bittybuzzVm(m_bytecode,
                      m_stringResolver,
                      BittyBuzzContainer::getBBZMessageHandler(),
                      BittyBuzzContainer::getBBZClosureRegister(),
                      BittyBuzzContainer::getBBZMessageService(),
                      BittyBuzzContainer::getBBZNeighborsManager(),
                      BSPContainer::getBSP(),
                      m_logger,
                      BSPContainer::getUserInterface()) {}

    ~BittyBuzzTask() override = default;

  private:
    ILogger& m_logger;
    BittyBuzzBytecode m_bytecode;
    BittyBuzzStringResolver m_stringResolver;
    BittyBuzzVm m_bittybuzzVm;

    void task() override {
        auto bbzFunctions = BittyBuzzFactory::createBittyBuzzGlobalLib();
        auto mathLib = BittyBuzzFactory::createBittyBuzzMathLib();
        std::array<std::reference_wrapper<IBittyBuzzLib>, 2> buzzLibraries{{bbzFunctions, mathLib}};
        if (!m_bittybuzzVm.init(buzzLibraries.data(), buzzLibraries.size())) {
            m_logger.log(LogLevel::Error, "BBZVM failed to initialize. state: %s err: %s",
                         BittyBuzzSystem::getStateString(m_bittybuzzVm.getState()),
                         BittyBuzzSystem::getErrorString(m_bittybuzzVm.getError()));
            return;
        }

        while (true) {

            if (!m_bittybuzzVm.step()) {
                m_logger.log(LogLevel::Error, "BBZVM failed to step. state: %s err: %s",
                             BittyBuzzSystem::getStateString(m_bittybuzzVm.getState()),
                             BittyBuzzSystem::getErrorString(m_bittybuzzVm.getError()));
            }
            Task::delay(100);
        }
    }
};

class MessageDispatcherTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    MessageDispatcherTask(const char* taskName,
                          UBaseType_t priority,
                          ICommInterface* stream,
                          ICircularQueue<MessageDTO>& streamQueue) :
        AbstractTask(taskName, priority),
        m_taskName(taskName),
        m_stream(stream),
        m_streamQueue(streamQueue),
        m_logger(LoggerContainer::getLogger()) {}

    ~MessageDispatcherTask() override = default;

    void setStream(ICommInterface* stream) { m_stream = stream; }

  private:
    const char* m_taskName;
    ICommInterface* m_stream;
    ICircularQueue<MessageDTO>& m_streamQueue;
    ILogger& m_logger;

    void task() override {
        if (m_stream != NULL) {

            HiveMindHostDeserializer deserializer(*m_stream);
            HiveMindHostAccumulatorSerializer serializer(*m_stream);
            HiveMindHostApiRequestHandler hivemindApiReqHandler =
                MessageHandlerContainer::createHiveMindHostApiRequestHandler();

            GreetSender greetSender(m_streamQueue, BSPContainer::getBSP());
            MessageDispatcher messageDispatcher = MessageHandlerContainer::createMessageDispatcher(
                deserializer, hivemindApiReqHandler, greetSender);

            while (m_stream->isConnected()) {
                if (!messageDispatcher.deserializeAndDispatch()) {
                    m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch to %s", m_taskName);
                }
            }
        }
    }
};

class MessageSenderTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    MessageSenderTask(const char* taskName,
                      UBaseType_t priority,
                      ICommInterface* stream,
                      INotificationQueue<MessageDTO>& streamQueue) :
        AbstractTask(taskName, priority),
        m_taskName(taskName),
        m_stream(stream),
        m_streamQueue(streamQueue),
        m_logger(LoggerContainer::getLogger()) {}

    ~MessageSenderTask() override = default;

    void setStream(ICommInterface* stream) { m_stream = stream; }

  private:
    const char* m_taskName;
    ICommInterface* m_stream;
    INotificationQueue<MessageDTO>& m_streamQueue;
    ILogger& m_logger;

    void task() override {
        if (m_stream != NULL) {
            HiveMindHostSerializer serializer(*m_stream);
            MessageSender messageSender(m_streamQueue, serializer, BSPContainer::getBSP(),
                                        m_logger);
            while (m_stream->isConnected()) {
                // Verify that we have a message to process
                if (m_streamQueue.isEmpty()) {
                    m_streamQueue.wait(500);
                }
                if (!messageSender.processAndSerialize()) {
                    m_logger.log(LogLevel::Warn, "Fail to process/serialize in %s", m_taskName);
                }
            }
        }
    }
};

class CommMonitoringTask : public AbstractTask<5 * configMINIMAL_STACK_SIZE> {
  public:
    CommMonitoringTask(const char* taskName,
                       UBaseType_t priority,
                       MessageDispatcherTask& dispatcherTask,
                       MessageSenderTask& senderTask,
                       CommInterfaceGetter commInterfaceGetter) :
        AbstractTask(taskName, priority),
        m_dispatcherTask(dispatcherTask),
        m_senderTask(senderTask),
        m_commInterfaceGetter(commInterfaceGetter),
        m_logger(LoggerContainer::getLogger()) {}

  private:
    MessageDispatcherTask& m_dispatcherTask;
    MessageSenderTask& m_senderTask;
    CommInterfaceGetter m_commInterfaceGetter;
    ILogger& m_logger;

    void task() override {
        while (true) {
            // TODO use notification instead of polling, need to add it in propolis os
            if (!m_dispatcherTask.isRunning() && !m_senderTask.isRunning()) {
                auto commInterfaceOpt = m_commInterfaceGetter();
                if (commInterfaceOpt) {
                    ICommInterface& commInterface = commInterfaceOpt.value();

                    if (commInterface.isConnected()) {

                        HiveMindHostSerializer serializer(commInterface);
                        HiveMindHostDeserializer deserializer(commInterface);
                        GreetHandler greetHandler(serializer, deserializer, BSPContainer::getBSP());

                        // Handshake
                        if (greetHandler.greet()) {
                            m_logger.log(LogLevel::Info, "Greet succeeded");
                            // Restart the tasks with the new streams
                            m_dispatcherTask.setStream(&commInterface);
                            m_senderTask.setStream(&commInterface);
                            m_dispatcherTask.start();
                            m_senderTask.start();
                        }
                    }
                }
            }

            Task::delay(1000);
        }
    }
};

class HardwareInterlocTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    HardwareInterlocTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~HardwareInterlocTask() override = default;

  private:
    void task() override { BSPContainer::getInterlocManager().startInterloc(); }
};

class SoftwareInterlocTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    SoftwareInterlocTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_interloc(InterlocContainer::getInterloc()),
        m_interlocMessageQueue(MessageHandlerContainer::getInterlocMsgQueue()),
        m_interlocMessageHandler(InterlocContainer::getInterlocMessageHandler()) {}

    ~SoftwareInterlocTask() override = default;

  private:
    // Create the object so it can register it's callbacks
    // TODO: remove once it is used somewhere else
    IInterloc& m_interloc;
    NotificationQueue<MessageDTO>& m_interlocMessageQueue;
    IInterlocMessageHandler& m_interlocMessageHandler;

    void task() override {
        while (true) {
            // Verify that we have a message to process
            if (m_interlocMessageQueue.isEmpty()) {
                m_interlocMessageQueue.wait(500);
            }
            m_interlocMessageHandler.processMessage();
        }
    }
};

class LogInterlocTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    LogInterlocTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_interloc(InterlocContainer::getInterloc()),
        m_logger(LoggerContainer::getLogger()) {}

    ~LogInterlocTask() override = default;

  private:
    IInterloc& m_interloc;
    ILogger& m_logger;

    void task() override {
        while (true) {
            PositionsTable interlocData = m_interloc.getPositionsTable();
            for (unsigned int i = 0; i < interlocData.m_positionsLength; i++) {
                m_logger.log(LogLevel::Info, "*****Distance from %d : %3.3f m",
                             interlocData.m_positions[i].m_robotId,
                             interlocData.m_positions[i].m_distance);
            }
            Task::delay(1000);
        }
    }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static BittyBuzzTask s_bittybuzzTask("bittybuzz", gc_taskNormalPriority);
    static HardwareInterlocTask s_hardwareInterlocTask("hardware_interloc", gc_taskHighPriority);
    static SoftwareInterlocTask s_softwareInterlocTask("software_interloc", gc_taskNormalPriority);
    static LogInterlocTask s_logInterlocTask("software_interloc", gc_taskNormalPriority);

    static MessageDispatcherTask s_hostDispatchTask("tcp_dispatch", gc_taskNormalPriority, NULL,
                                                    MessageHandlerContainer::getHostMsgQueue());
    static MessageSenderTask s_hostMessageSender("host_send", gc_taskNormalPriority, NULL,
                                                 MessageHandlerContainer::getHostMsgQueue());
    static MessageDispatcherTask s_remoteDispatchTask("remote_dispatch", gc_taskNormalPriority,
                                                      NULL,
                                                      MessageHandlerContainer::getRemoteMsgQueue());
    static MessageSenderTask s_remoteMessageSender("remote_send", gc_taskNormalPriority, NULL,
                                                   MessageHandlerContainer::getRemoteMsgQueue());

    static CommMonitoringTask s_hostMonitorTask("host_monitor", gc_taskNormalPriority,
                                                s_hostDispatchTask, s_hostMessageSender,
                                                BSPContainer::getHostCommInterface);
    static CommMonitoringTask s_remoteMonitorTask("remote_monitor", gc_taskNormalPriority,
                                                  s_remoteDispatchTask, s_remoteMessageSender,
                                                  BSPContainer::getRemoteCommInterface);

    s_bittybuzzTask.start();
    s_hardwareInterlocTask.start();
    s_softwareInterlocTask.start();
    s_logInterlocTask.start();
    s_hostMonitorTask.start();
    s_remoteMonitorTask.start();

    Task::startScheduler();

    return 0;
}
