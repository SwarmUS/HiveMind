#include <AbstractTask.h>
#include <BaseTask.h>
#include <Task.h>
#include <application-interface/ApplicationInterfaceContainer.h>
#include <atomic>
#include <bittybuzz/BittyBuzzContainer.h>
#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzSystem.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <bsp/SettingsContainer.h>
#include <cstdlib>
#include <interloc/IInterloc.h>
#include <interloc/InterlocContainer.h>
#include <logger/Logger.h>
#include <logger/LoggerContainer.h>
#include <message-handler/GreetHandler.h>
#include <message-handler/GreetSender.h>
#include <message-handler/MessageDispatcher.h>
#include <message-handler/MessageHandlerContainer.h>
#include <message-handler/MessageSender.h>
#include <pheromones/HiveMindHostAccumulatorSerializer.h>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>

constexpr uint16_t gc_taskNormalPriority = tskIDLE_PRIORITY + 1;
constexpr uint16_t gc_taskHighPriority = tskIDLE_PRIORITY + 30; // Higher priority then LwIP

// Need to return the proper comm interface
using commInterfaceGetter = std::optional<std::reference_wrapper<ICommInterface>> (*)();

// Set connection state
using connectionStateSetter = void (*)(ConnectionState state);

class BittyBuzzTask : public AbstractTask<20 * configMINIMAL_STACK_SIZE> {
  public:
    BittyBuzzTask(const char* taskName,
                  UBaseType_t priority,
                  IDeviceStateUI& deviceStateUI,
                  IButtonCallbackRegister& buttonCallbackRegister) :

        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_deviceStateUI(deviceStateUI),
        m_buttonCallbackRegister(buttonCallbackRegister),
        m_bytecode(BittyBuzzFactory::createBittyBuzzBytecode(m_logger)),
        m_stringResolver(BittyBuzzFactory::createBittyBuzzStringResolver(m_logger)),
        m_bittybuzzVm(m_bytecode,
                      m_stringResolver,
                      BittyBuzzContainer::getBBZMessageHandler(),
                      BittyBuzzContainer::getBBZClosureRegister(),
                      BittyBuzzContainer::getBBZMessageService(),
                      BittyBuzzContainer::getBBZNeighborsManager(),
                      ApplicationInterfaceContainer::getUserUI(),
                      BSPContainer::getBSP(),
                      m_logger,
                      BSPContainer::getUserInterface()) {}

    ~BittyBuzzTask() override = default;

  private:
    ILogger& m_logger;
    IDeviceStateUI& m_deviceStateUI;
    IButtonCallbackRegister& m_buttonCallbackRegister;
    BittyBuzzBytecode m_bytecode;
    BittyBuzzStringResolver m_stringResolver;
    BittyBuzzVm m_bittybuzzVm;

    std::atomic_bool m_resetVm = false;

    static DeviceState vmErrorToDeviceState(bbzvm_error err) {
        return static_cast<DeviceState>(err);
    }

    static void resetVmButtonCallback(void* context) {
        BittyBuzzTask* bbvmTask = (BittyBuzzTask*)context;
        bbvmTask->m_resetVm = true;
        bbvmTask->m_bittybuzzVm.stop(); // Stopping the vm so it can be terminated and started again
    }

    void task() override {
        const int16_t stepDelay = (int16_t)SettingsContainer::getBBZVMStepDelayMs();
        auto globalLib = BittyBuzzFactory::createBittyBuzzGlobalLib(stepDelay);
        auto mathLib = BittyBuzzFactory::createBittyBuzzMathLib();
        auto uiLib = BittyBuzzFactory::createBittyBuzzUILib();

        std::array<std::reference_wrapper<IBittyBuzzLib>, 3> buzzLibraries{
            {globalLib, mathLib, uiLib}};

        // register btn function
        m_buttonCallbackRegister.setCallback(resetVmButtonCallback, this);

        Task::Time prevWake = Task::getTime();

        while (true) {
            m_deviceStateUI.setDeviceState(DeviceState::Ok);

            if (!m_bittybuzzVm.init(buzzLibraries.data(), buzzLibraries.size()) ||
                !m_bittybuzzVm.start()) {

                m_logger.log(LogLevel::Error, "BBZVM failed to initialize. state: %s err: %s",
                             BittyBuzzSystem::getStateString(m_bittybuzzVm.getState()),
                             BittyBuzzSystem::getErrorString(m_bittybuzzVm.getError()));
                m_deviceStateUI.setDeviceState(vmErrorToDeviceState(m_bittybuzzVm.getError()));
                return;
            }

            while (!m_resetVm) {
                if (m_bittybuzzVm.getState() == BBZVM_STATE_READY) {
                    BBVMRet statusCode = m_bittybuzzVm.step();
                    switch (statusCode) {
                    case BBVMRet::Ok:
                        m_deviceStateUI.setDeviceState(DeviceState::Ok);
                        break;
                    case BBVMRet::Stopped: {
                        m_logger.log(LogLevel::Warn, "BBZVM is stopped, cannot step");
                        m_deviceStateUI.setDeviceState(DeviceState::Error);
                        break;
                    }
                    case BBVMRet::OutMsgErr: {
                        m_logger.log(LogLevel::Warn, "Buzz could not sent message");
                        m_deviceStateUI.setDeviceState(DeviceState::Error);
                        break;
                    }
                    case BBVMRet::VmErr: {
                        m_logger.log(LogLevel::Error, "BBZVM failed to step. state: %s err: %s",
                                     BittyBuzzSystem::getStateString(m_bittybuzzVm.getState()),
                                     BittyBuzzSystem::getErrorString(m_bittybuzzVm.getError()));
                        m_deviceStateUI.setDeviceState(
                            vmErrorToDeviceState(m_bittybuzzVm.getError()));
                        break;
                    }
                    }
                }
                Task::delayUntil(prevWake, (uint16_t)stepDelay);
            }
            // VM needs to be resetted so we terminate it and init it again
            m_resetVm = false;
            m_logger.log(LogLevel::Warn, "VM State before reset:");
            m_bittybuzzVm.logDump(LogLevel::Warn);
            m_logger.log(LogLevel::Warn, "Resetting VM");
            m_bittybuzzVm.terminate();
        }
    }
};

class MessageDispatcherTask : public AbstractTask<15 * configMINIMAL_STACK_SIZE> {
  public:
    MessageDispatcherTask(const char* taskName,
                          UBaseType_t priority,
                          ICommInterface* stream,
                          ICircularQueue<MessageDTO>& streamQueue) :
        AbstractTask(taskName, priority),
        m_stream(stream),
        m_streamQueue(streamQueue),
        m_logger(LoggerContainer::getLogger()) {}

    ~MessageDispatcherTask() override = default;

    void setStream(ICommInterface* stream) { m_stream = stream; }

  private:
    ICommInterface* m_stream;
    ICircularQueue<MessageDTO>& m_streamQueue;
    ILogger& m_logger;

    void task() override {
        if (m_stream != NULL) {

            HiveMindHostDeserializer deserializer(*m_stream);
            HiveMindHostAccumulatorSerializer serializer(*m_stream);
            HiveMindHostApiRequestHandler hivemindApiReqHandler =
                MessageHandlerContainer::createHiveMindHostApiRequestHandler();
            HiveConnectHiveMindApiMessageHandler hiveconnectApiMessageHandler =
                MessageHandlerContainer::createHiveConnectHiveMindApiMessageHandler();

            GreetSender greetSender(m_streamQueue, BSPContainer::getBSP());
            MessageDispatcher messageDispatcher = MessageHandlerContainer::createMessageDispatcher(
                deserializer, hivemindApiReqHandler, hiveconnectApiMessageHandler, greetSender);

            while (m_stream->isConnected()) {
                if (!messageDispatcher.deserializeAndDispatch()) {
                    m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch to %s", m_taskName);
                }
            }
        }
        m_logger.log(LogLevel::Warn, "Exiting %s task", m_taskName);
    }
};

class MessageSenderTask : public AbstractTask<15 * configMINIMAL_STACK_SIZE> {
  public:
    MessageSenderTask(const char* taskName,
                      UBaseType_t priority,
                      ICommInterface* stream,
                      INotificationQueue<MessageDTO>& streamQueue,
                      bool keepRunning = false) :
        AbstractTask(taskName, priority),
        m_stream(stream),
        m_streamQueue(streamQueue),
        m_logger(LoggerContainer::getLogger()),
        m_keepRunning(keepRunning) {}

    ~MessageSenderTask() override = default;

    void setStream(ICommInterface* stream) { m_stream = stream; }
    void setSerializer(IHiveMindHostSerializer* serializer) { m_serializer = serializer; }

  private:
    ICommInterface* m_stream;
    INotificationQueue<MessageDTO>& m_streamQueue;
    ILogger& m_logger;
    IHiveMindHostSerializer* m_serializer = NULL;
    bool m_keepRunning;

    void task() override {
        if (m_stream != NULL && m_serializer != NULL) {
            MessageSender messageSender(m_streamQueue, *m_serializer, BSPContainer::getBSP(),
                                        m_logger);
            while (m_stream->isConnected() || m_keepRunning) {
                if (m_stream->isConnected()) {
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
        m_logger.log(LogLevel::Warn, "Exiting %s task", m_taskName);
    }
};

template <typename SerializerType = HiveMindHostSerializer>
class CommMonitoringTask : public AbstractTask<15 * configMINIMAL_STACK_SIZE> {
  public:
    CommMonitoringTask<SerializerType>(const char* taskName,
                                       UBaseType_t priority,
                                       MessageDispatcherTask& dispatcherTask,
                                       MessageSenderTask& senderTask,
                                       IHandshakeUI& handshakeUI,
                                       commInterfaceGetter commInterfaceGetter,
                                       connectionStateSetter connectionStateSetter) :
        AbstractTask(taskName, priority),
        m_dispatcherTask(dispatcherTask),
        m_senderTask(senderTask),
        m_handshakeUI(handshakeUI),
        m_commInterfaceGetter(commInterfaceGetter),
        m_connectionStateSetter(connectionStateSetter),
        m_logger(LoggerContainer::getLogger()) {}

  private:
    MessageDispatcherTask& m_dispatcherTask;
    MessageSenderTask& m_senderTask;
    IHandshakeUI& m_handshakeUI;
    commInterfaceGetter m_commInterfaceGetter;
    connectionStateSetter m_connectionStateSetter;
    ILogger& m_logger;

    void task() override {
        while (true) {

            m_handshakeUI.handshake(false);
            // TODO use notification instead of polling, need to add it in
            // propolis os
            if (!m_dispatcherTask.isRunning() && !m_senderTask.isRunning()) {
                auto commInterfaceOpt = m_commInterfaceGetter();
                if (commInterfaceOpt) {
                    ICommInterface& commInterface = commInterfaceOpt.value();

                    if (commInterface.isConnected()) {

                        HiveMindHostDeserializer deserializer(commInterface);
                        SerializerType serializer(commInterface);
                        GreetHandler greetHandler(serializer, deserializer, BSPContainer::getBSP());
                        m_senderTask.setSerializer(&serializer);

                        // Handshake
                        if (greetHandler.greet()) {
                            m_handshakeUI.handshake(true);
                            m_logger.log(LogLevel::Info, "Greet succeeded in %s", m_taskName);
                            // Restart the tasks with the new streams
                            m_dispatcherTask.setStream(&commInterface);
                            m_senderTask.setStream(&commInterface);
                            m_dispatcherTask.start();
                            m_senderTask.start();

                            while (m_dispatcherTask.isRunning() || m_senderTask.isRunning()) {
                                Task::delay(1000);
                                if (m_dispatcherTask.isRunning() != m_senderTask.isRunning()) {
                                    const char* taskStuckRuning =
                                        m_dispatcherTask.isRunning()
                                            ? m_dispatcherTask.getTaskName()
                                            : m_senderTask.getTaskName();
                                    const char* taskEndedRuning =
                                        !m_dispatcherTask.isRunning()
                                            ? m_dispatcherTask.getTaskName()
                                            : m_senderTask.getTaskName();
                                    m_logger.log(LogLevel::Warn,
                                                 "%s: Task %s is running while %s is not",
                                                 m_taskName, taskStuckRuning, taskEndedRuning);
                                    m_connectionStateSetter(ConnectionState::Error);
                                }
                            }
                        }
                        m_handshakeUI.handshake(false);
                    }
                }
            }
            Task::delay(1000);
        }
    }
};

class HardwareInterlocTask : public AbstractTask<20 * configMINIMAL_STACK_SIZE> {
  public:
    HardwareInterlocTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~HardwareInterlocTask() override = default;

  private:
    void task() override { BSPContainer::getInterlocManager().startInterloc(); }
};

class InterlocMessageHandlerTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    InterlocMessageHandlerTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_interlocMessageQueue(MessageHandlerContainer::getInterlocMsgQueue()),
        m_interlocMessageHandler(InterlocContainer::getInterlocMessageHandler()) {}

    ~InterlocMessageHandlerTask() override = default;

  private:
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

class InterlocDataHandlerTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    InterlocDataHandlerTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_interloc(InterlocContainer::getInterloc()) {}

    ~InterlocDataHandlerTask() override = default;

  private:
    IInterloc& m_interloc;

    void task() override {
        while (true) {
            m_interloc.process();
        }
    }
};

class LogInterlocTask : public AbstractTask<4 * configMINIMAL_STACK_SIZE> {
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
                m_logger.log(LogLevel::Info, "*****Distance from %d : %3.3f m, angle: %3.3f",
                             interlocData.m_positions[i].m_robotId,
                             interlocData.m_positions[i].m_distance,
                             interlocData.m_positions[i].m_angle);
            }
            Task::delay(1000);
        }
    }
};

// Gets the comm interface for the host and sets the UI for wich one was obtained
std::optional<std::reference_wrapper<ICommInterface>> hostInterfaceGetter() {
    auto commInterface = BSPContainer::getHostCommInterface();
    auto connectionStateUI = ApplicationInterfaceContainer::getConnectionStateUI();
    connectionStateUI.setConnectionState(ConnectionState::Unconnected);

    if (commInterface) {
        ConnectionType type = commInterface.value().get().getType();
        switch (type) {
        case ConnectionType::Ethernet:
            connectionStateUI.setConnectionState(ConnectionState::Ethernet);
            break;
        case ConnectionType::USB:
            connectionStateUI.setConnectionState(ConnectionState::USB);
            break;
        default:
            connectionStateUI.setConnectionState(ConnectionState::Error);
        }
    }
    return commInterface;
}

void setHostConnectionState(ConnectionState state) {
    auto connectionStateUI = ApplicationInterfaceContainer::getConnectionStateUI();
    connectionStateUI.setConnectionState(state);
}

void setRemoteConnectionState(ConnectionState state) {
    LoggerContainer::getLogger().log(LogLevel::Info, "Remote connection state set to %d", state);
}

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    ApplicationInterfaceContainer::getConnectionStateUI().setConnectionState(
        ConnectionState::Booting);

    __attribute__((section(".cmbss"))) static BittyBuzzTask s_bittybuzzTask(
        "bittybuzz", gc_taskNormalPriority, ApplicationInterfaceContainer::getDeviceStateUI(),
        ApplicationInterfaceContainer::getButton1CallbackRegister());

    static HardwareInterlocTask s_hardwareInterlocTask("hardware_interloc", gc_taskHighPriority);
    static InterlocMessageHandlerTask s_interlocMessageTask("interloc_message_handler",
                                                            gc_taskNormalPriority);
    static InterlocDataHandlerTask s_interlocDataTask("interloc_data_handler",
                                                      gc_taskNormalPriority);
    static LogInterlocTask s_logInterlocTask("software_interloc_log", gc_taskNormalPriority);

    static MessageDispatcherTask s_hostDispatchTask("host_dispatch", gc_taskNormalPriority, NULL,
                                                    MessageHandlerContainer::getHostMsgQueue());
    static MessageSenderTask s_hostMessageSender("host_send", gc_taskNormalPriority, NULL,
                                                 MessageHandlerContainer::getHostMsgQueue());

    static MessageDispatcherTask s_remoteDispatchTask("remote_dispatch", gc_taskNormalPriority,
                                                      NULL,
                                                      MessageHandlerContainer::getRemoteMsgQueue());
    static MessageSenderTask s_remoteMessageSender("remote_send", gc_taskNormalPriority, NULL,
                                                   MessageHandlerContainer::getRemoteMsgQueue(),
                                                   true);

    static CommMonitoringTask s_hostMonitorTask("host_monitor", gc_taskNormalPriority,
                                                s_hostDispatchTask, s_hostMessageSender,
                                                ApplicationInterfaceContainer::getHostHandshakeUI(),
                                                hostInterfaceGetter, setHostConnectionState);

    static CommMonitoringTask<HiveMindHostAccumulatorSerializer> s_remoteMonitorTask(
        "remote_monitor", gc_taskNormalPriority, s_remoteDispatchTask, s_remoteMessageSender,
        ApplicationInterfaceContainer::getRemoteHandshakeUI(), BSPContainer::getRemoteCommInterface,
        setRemoteConnectionState);

    s_bittybuzzTask.start();
    s_hardwareInterlocTask.start();
    s_interlocDataTask.start();
    s_interlocMessageTask.start();
    s_logInterlocTask.start();
    s_hostMonitorTask.start();
    s_remoteMonitorTask.start();
    Task::startScheduler();

    return 0;
}
