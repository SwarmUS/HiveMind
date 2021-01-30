if(NOT LWIP_FIND_COMPONENTS)
    set(LWIP_FIND_COMPONENTS
            STM32F0 STM32F1 STM32F2 STM32F3 STM32F4 STM32F7
            STM32G0 STM32G4
            STM32H7_M7 STM32H7_M4
            STM32L0 STM32L1 STM32L4
            )
endif()

if(STM32H7 IN_LIST LWIP_FIND_COMPONENTS)
    list(REMOVE_ITEM LWIP_FIND_COMPONENTS STM32H7)
    list(APPEND LWIP_FIND_COMPONENTS STM32H7_M7 STM32H7_M4)
endif()
list(REMOVE_DUPLICATES LWIP_FIND_COMPONENTS)

include(stm32/devices)

foreach(COMP ${LWIP_FIND_COMPONENTS})
    string(TOLOWER ${COMP} COMP_L)
    string(TOUPPER ${COMP} COMP)

    string(REGEX MATCH "^STM32([A-Z][0-9])([0-9A-Z][0-9][A-Z][0-9A-Z])?_?(M[47])?.*$" COMP ${COMP})

    if((NOT CMAKE_MATCH_1) AND (NOT CMAKE_MATCH_2))
        message(FATAL_ERROR "Unknown CMSIS component: ${COMP}")
    endif()

    if(CMAKE_MATCH_2)
        set(FAMILY ${CMAKE_MATCH_1})
        set(DEVICES "${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
    else()
        set(FAMILY ${CMAKE_MATCH_1})
        stm32_get_devices_by_family(DEVICES FAMILY ${FAMILY} CORE ${CORE})
    endif()

    if(CMAKE_MATCH_3)
        set(CORE ${CMAKE_MATCH_3})
        set(CORE_C "::${CORE}")
        set(CORE_U "_${CORE}")
    else()
        unset(CORE)
        unset(CORE_C)
        unset(CORE_U)
    endif()

    string(TOLOWER ${FAMILY} FAMILY_L)

    if((NOT STM32_CMSIS_${FAMILY}_PATH) AND (NOT STM32_CUBE_${FAMILY}_PATH))
        set(STM32_CUBE_${FAMILY}_PATH /opt/STM32Cube${FAMILY} CACHE PATH "Path to STM32Cube${FAMILY}")
        message(STATUS "Neither STM32_CUBE_${FAMILY}_PATH nor STM32_CMSIS_${FAMILY}_PATH specified using default  STM32_CUBE_${FAMILY}_PATH: ${STM32_CUBE_${FAMILY}_PATH}")
    endif()


    find_path(CMSIS_${FAMILY}${CORE_U}_LWIP_DIR
            NAMES CMakeLists.txt
            PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/Third_Party/LwIP"
            NO_DEFAULT_PATH
            )

    ############### EXTRACTED FROM Filelists.cmake IN LWIP DIR ##############################
    set(LWIP_VERSION_MAJOR    "2")
    set(LWIP_VERSION_MINOR    "1")
    set(LWIP_VERSION_REVISION "2")
    # LWIP_VERSION_RC is set to LWIP_RC_RELEASE for official releases
    # LWIP_VERSION_RC is set to LWIP_RC_DEVELOPMENT for Git versions
    # Numbers 1..31 are reserved for release candidates
    set(LWIP_VERSION_RC       "LWIP_RC_RELEASE")

    if ("${LWIP_VERSION_RC}" STREQUAL "LWIP_RC_RELEASE")
        set(LWIP_VERSION_STRING
                "${LWIP_VERSION_MAJOR}.${LWIP_VERSION_MINOR}.${LWIP_VERSION_REVISION}"
                )
    elseif ("${LWIP_VERSION_RC}" STREQUAL "LWIP_RC_DEVELOPMENT")
        set(LWIP_VERSION_STRING
                "${LWIP_VERSION_MAJOR}.${LWIP_VERSION_MINOR}.${LWIP_VERSION_REVISION}.dev"
                )
    else ("${LWIP_VERSION_RC}" STREQUAL "LWIP_RC_RELEASE")
        set(LWIP_VERSION_STRING
                "${LWIP_VERSION_MAJOR}.${LWIP_VERSION_MINOR}.${LWIP_VERSION_REVISION}.rc${LWIP_VERSION_RC}"
                )
    endif ("${LWIP_VERSION_RC}" STREQUAL "LWIP_RC_RELEASE")

    configure_file(${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/include/lwip/init.h.cmake.in ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/include/lwip/init.h)

    set(lwipcore_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/init.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/def.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/dns.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/inet_chksum.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ip.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/mem.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/memp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/netif.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/pbuf.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/raw.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/stats.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/sys.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/altcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/altcp_alloc.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/altcp_tcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/tcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/tcp_in.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/tcp_out.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/timeouts.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/udp.c
            )
    set(lwipcore4_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/autoip.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/dhcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/etharp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/icmp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/igmp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/ip4_frag.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/ip4.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv4/ip4_addr.c
            )
    set(lwipcore6_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/dhcp6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/ethip6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/icmp6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/inet6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/ip6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/ip6_addr.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/ip6_frag.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/mld6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/core/ipv6/nd6.c
            )

    # APIFILES: The files which implement the sequential and socket APIs.
    set(lwipapi_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/api_lib.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/api_msg.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/err.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/if_api.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/netbuf.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/netdb.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/netifapi.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/sockets.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/api/tcpip.c
            )

    # Files implementing various generic network interface functions
    set(lwipnetif_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ethernet.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/bridgeif.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/bridgeif_fdb.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/slipif.c
            )

    # 6LoWPAN
    set(lwipsixlowpan_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/lowpan6_common.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/lowpan6.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/lowpan6_ble.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/zepif.c
            )

    # PPP
    set(lwipppp_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/auth.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/ccp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/chap-md5.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/chap_ms.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/chap-new.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/demand.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/eap.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/ecp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/eui64.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/fsm.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/ipcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/ipv6cp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/lcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/magic.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/mppe.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/multilink.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/ppp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/pppapi.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/pppcrypt.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/pppoe.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/pppol2tp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/pppos.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/upap.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/utils.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/vj.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/polarssl/arc4.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/polarssl/des.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/polarssl/md4.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/polarssl/md5.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/netif/ppp/polarssl/sha1.c
            )

    # SNMPv3 agent
    set(lwipsnmp_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_asn1.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_core.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_icmp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_interfaces.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_ip.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_snmp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_system.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_tcp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_mib2_udp.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_snmpv2_framework.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_snmpv2_usm.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_msg.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmpv3.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_netconn.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_pbuf_stream.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_raw.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_scalar.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_table.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_threadsync.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmp_traps.c
            )

    # HTTP server + client
    set(lwiphttp_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/http/altcp_proxyconnect.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/http/fs.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/http/http_client.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/http/httpd.c
            )

    # MAKEFSDATA HTTP server host utility
    set(lwipmakefsdata_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/http/makefsdata/makefsdata.c
            )

    # IPERF server
    set(lwipiperf_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/lwiperf/lwiperf.c
            )

    # SMTP client
    set(lwipsmtp_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/smtp/smtp.c
            )

    # SNTP client
    set(lwipsntp_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/sntp/sntp.c
            )

    # MDNS responder
    set(lwipmdns_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/mdns/mdns.c
            )

    # NetBIOS name server
    set(lwipnetbios_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/netbiosns/netbiosns.c
            )

    # TFTP server files
    set(lwiptftp_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/tftp/tftp_server.c
            )

    # MQTT client files
    set(lwipmqtt_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/mqtt/mqtt.c
            )

    # ARM MBEDTLS related files of lwIP rep
    set(lwipmbedtls_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/altcp_tls/altcp_tls_mbedtls.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/altcp_tls/altcp_tls_mbedtls_mem.c
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/apps/snmp/snmpv3_mbedtls.c
            )

    set(lwipsys_SRCS
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/system/OS/sys_arch.c
            )

    set(lwip_INC
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/src/include
            ${CMSIS_${FAMILY}${CORE_U}_LWIP_DIR}/system
            )

    # All LWIP files without apps
    set(lwipnoapps_SRCS
            ${lwipcore_SRCS}
            ${lwipcore4_SRCS}
            ${lwipcore6_SRCS}
            ${lwipapi_SRCS}
            ${lwipnetif_SRCS}
            ${lwipsixlowpan_SRCS}
            ${lwipppp_SRCS}
            ${lwipsys_SRCS}
            )

    # LWIPAPPFILES: All LWIP APPs
    set(lwipallapps_SRCS
            ${lwipsnmp_SRCS}
            ${lwiphttp_SRCS}
            ${lwipiperf_SRCS}
            ${lwipsmtp_SRCS}
            ${lwipsntp_SRCS}
            ${lwipmdns_SRCS}
            ${lwipnetbios_SRCS}
            ${lwiptftp_SRCS}
            ${lwipmqtt_SRCS}
            ${lwipmbedtls_SRCS}
            )

    ########################################################################################################

    if (NOT (TARGET CMSIS::STM32::${FAMILY}${CORE_C}::LWIP))
        add_library(CMSIS::STM32::${FAMILY}${CORE_C}::LWIP INTERFACE IMPORTED)
        target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C}::LWIP INTERFACE "${lwip_INC}")
        target_sources(CMSIS::STM32::${FAMILY}${CORE_C}::LWIP INTERFACE "${lwipnoapps_SRCS}")
    endif()

    list(APPEND LWIP_SRC_DIRS ${lwipnoapps_SRCS})
    set(LWIP_${COMP}_FOUND true)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LWIP
        REQUIRED_VARS LWIP_SRC_DIRS
        FOUND_VAR LWIP_FOUND
        HANDLE_COMPONENTS
        )
