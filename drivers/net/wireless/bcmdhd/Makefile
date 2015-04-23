# bcmdhd
#####################
# SDIO Basic feature
#####################

DHDCFLAGS += -Wall -Wstrict-prototypes -Dlinux -DLINUX -DBCMDRIVER            \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DSHOW_EVENTS -DBCMDBG -DWLP2P                            \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DCSCAN -DPKT_FILTER_SUPPORT                             \
	-DEMBEDDED_PLATFORM -DPNO_SUPPORT  -DSHOW_LOGTRACE                    \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT                           \
	-DCUSTOMER_HW2 -DGET_CUSTOM_MAC_ENABLE

#################
# Common feature
#################
DHDCFLAGS += -DWL_CFG80211
# Print out kernel panic point of file and line info when assertion happened
DHDCFLAGS += -DBCMASSERT_LOG

# keepalive
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=28000

DHDCFLAGS += -DVSDB

# For p2p connection issue
DHDCFLAGS += -DWL_SCB_TIMEOUT=10


# TDLS enable
DHDCFLAGS += -DWLTDLS -DWLTDLS_AUTO_ENABLE
# For TDLS tear down inactive time 40 sec
DHDCFLAGS += -DCUSTOM_TDLS_IDLE_MODE_SETTING=40000
# for TDLS RSSI HIGH for establishing TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_HIGH=-60
# for TDLS RSSI HIGH for tearing down TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_LOW=-70

# Roaming
DHDCFLAGS += -DROAM_AP_ENV_DETECTION
DHDCFLAGS += -DROAM_ENABLE -DROAM_CHANNEL_CACHE -DROAM_API
DHDCFLAGS += -DENABLE_FW_ROAM_SUSPEND
# Roaming trigger
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-75
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=10
# Set PM 2 always regardless suspend/resume
DHDCFLAGS += -DSUPPORT_PM2_ONLY

# For special PNO Event keep wake lock for 10sec
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=10
DHDCFLAGS += -DMIRACAST_AMPDU_SIZE=8

#Gscan
DHDCFLAGS += -DGSCAN_SUPPORT
DHDCFLAGS += -DRTT_SUPPORT -DRTT_DEBUG
DHDCFLAGS += -DWL_VENDOR_EXT_SUPPORT
#Link Statistics
#DHDCFLAGS += -DLINKSTAT_SUPPORT


# Early suspend
DHDCFLAGS += -DDHD_USE_EARLYSUSPEND

# For Scan result patch
DHDCFLAGS += -DESCAN_RESULT_PATCH

# For Static Buffer
ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
  DHDCFLAGS += -DENHANCED_STATIC_BUF
  DHDCFLAGS += -DSTATIC_WL_PRIV_STRUCT
endif
DHDCFLAGS += -DWL_SCHED_SCAN
ccflags-y += -DWL_SCHED_SCAN

# Ioctl timeout 5000ms
DHDCFLAGS += -DIOCTL_RESP_TIMEOUT=5000

# Prevent rx thread monopolize
DHDCFLAGS += -DWAIT_DEQUEUE

# Config PM Control
DHDCFLAGS += -DCONFIG_CONTROL_PM

# idle count
DHDCFLAGS += -DDHD_USE_IDLECOUNT

# SKB TAILPAD to avoid out of boundary memory access
DHDCFLAGS += -DDHDENABLE_TAILPAD

# Wi-Fi Direct
DHDCFLAGS += -DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
DHDCFLAGS += -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DWL_IFACE_COMB_NUM_CHANNELS
DHDCFLAGS += -DWL_ENABLE_P2P_IF

DHDCFLAGS += -DWL_CFG80211_ACL
DHDCFLAGS += -DDISABLE_11H_SOFTAP
DHDCFLAGS += -DSET_RANDOM_MAC_SOFTAP
DHDCFLAGS += -DCUSTOM_FORCE_NODFS_FLAG
DHDCFLAGS += -DCUSTOM_SET_SHORT_DWELL_TIME

##########################
# driver type
# m: module type driver
# y: built-in type driver
##########################
DRIVER_TYPE ?= m

#########################
# Chip dependent feature
#########################

ifneq ($(filter y, $(CONFIG_BCM4354) $(CONFIG_BCM4356)),)
  DHDCFLAGS += -DUSE_WL_TXBF
  DHDCFLAGS += -DUSE_WL_FRAMEBURST
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
  DHDCFLAGS += -DMAX_AP_CLIENT_CNT=10
  DHDCFLAGS += -DMAX_GO_CLIENT_CNT=5

# New Features
  DHDCFLAGS += -DWL11U
  DHDCFLAGS += -DMFP
  DHDCFLAGS += -DDHD_ENABLE_LPC
  DHDCFLAGS += -DCUSTOM_COUNTRY_CODE
  DHDCFLAGS += -DSAR_SUPPORT

# debug info
  DHDCFLAGS += -DDHD_WAKE_STATUS

ifneq ($(CONFIG_BCMDHD_SDIO),)
  DHDCFLAGS += -DBDC -DOOB_INTR_ONLY -DHW_OOB -DDHD_BCMEVENTS -DMMC_SDIO_ABORT
  DHDCFLAGS += -DBCMSDIO -DBCMLXSDMMC -DUSE_SDIOFIFO_IOVAR
  DHDCFLAGS += -DPROP_TXSTATUS
  DHDCFLAGS += -DCUSTOM_AMPDU_MPDU=16
  DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=64
# tput enhancement
  DHDCFLAGS += -DCUSTOM_GLOM_SETTING=8 -DCUSTOM_RXCHAIN=1
  DHDCFLAGS += -DUSE_DYNAMIC_F2_BLKSIZE -DDYNAMIC_F2_BLKSIZE_FOR_NONLEGACY=128
  DHDCFLAGS += -DBCMSDIOH_TXGLOM -DCUSTOM_TXGLOM=1 -DBCMSDIOH_TXGLOM_HIGHSPEED
  DHDCFLAGS += -DDHDTCPACK_SUPPRESS
  DHDCFLAGS += -DRXFRAME_THREAD
  DHDCFLAGS += -DREPEAT_READFRAME
  DHDCFLAGS += -DCUSTOM_MAX_TXGLOM_SIZE=40
  DHDCFLAGS += -DMAX_HDR_READ=128
  DHDCFLAGS += -DDHD_FIRSTREAD=128

# bcn_timeout
  DHDCFLAGS += -DCUSTOM_BCN_TIMEOUT_SETTING=5

  DHDCFLAGS += -DWLFC_STATE_PREALLOC
endif

ifneq ($(CONFIG_BCMDHD_PCIE),)
  DHDCFLAGS += -DPCIE_FULL_DONGLE -DBCMPCIE -DCUSTOM_DPC_PRIO_SETTING=-1
# tput enhancement
  DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=64
  DHDCFLAGS += -DCUSTOM_AMPDU_MPDU=32
  DHDCFLAGS += -DCUSTOM_AMPDU_RELEASE=16
  DHDCFLAGS += -DPROP_TXSTATUS_VSDB
# Disable watchdog thread
  DHDCFLAGS += -DCUSTOM_DHD_WATCHDOG_MS=0

  DHDCFLAGS += -DMAX_CNTL_TX_TIMEOUT=1
ifneq ($(CONFIG_ARCH_MSM),)
  DHDCFLAGS += -DMSM_PCIE_LINKDOWN_RECOVERY
endif
ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
  DHDCFLAGS += -DDHD_USE_STATIC_IOCTLBUF
endif

  DHDCFLAGS += -DDONGLE_ENABLE_ISOLATION
endif

# Print 802.1X packets
  DHDCFLAGS += -DDHD_8021X_DUMP
# Print DHCP packets
#  DHDCFLAGS += -DDHD_DHCP_DUMP
endif

ifneq ($(CONFIG_BCM4339),)
  DHDCFLAGS += -DBCM4339_CHIP -DHW_OOB

  # tput enhancement
  DHDCFLAGS += -DCUSTOM_GLOM_SETTING=8 -DCUSTOM_RXCHAIN=1
  DHDCFLAGS += -DUSE_DYNAMIC_F2_BLKSIZE -DDYNAMIC_F2_BLKSIZE_FOR_NONLEGACY=128
  DHDCFLAGS += -DBCMSDIOH_TXGLOM -DCUSTOM_TXGLOM=1 -DBCMSDIOH_TXGLOM_HIGHSPEED
  DHDCFLAGS += -DDHDTCPACK_SUPPRESS
  DHDCFLAGS += -DUSE_WL_TXBF
  DHDCFLAGS += -DUSE_WL_FRAMEBURST
  DHDCFLAGS += -DRXFRAME_THREAD
  DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=64
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
  DHDCFLAGS += -DPROP_TXSTATUS_VSDB
  DHDCFLAGS += -DCUSTOM_MAX_TXGLOM_SIZE=32

  # New Features
  DHDCFLAGS += -DWL11U
  DHDCFLAGS += -DDHD_ENABLE_LPC
  DHDCFLAGS += -DCUSTOM_PSPRETEND_THR=30
endif


#EXTRA_LDFLAGS += --strip-debug

ifeq ($(DRIVER_TYPE),y)
  DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
  DHDCFLAGS += -DUSE_LATE_INITCALL_SYNC
endif
EXTRA_CFLAGS += $(DHDCFLAGS) -DDHD_DEBUG -DBCMSDIO
EXTRA_CFLAGS += -DSRCBASE=\"$(src)\"
EXTRA_CFLAGS += -I$(src)/include/ -I$(src)/
KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)


bcmdhd-objs += dhd_pno.o 
bcmdhd-objs += dhd_common.o
bcmdhd-objs += dhd_ip.o
bcmdhd-objs += dhd_custom_gpio.o
bcmdhd-objs += dhd_linux.o
bcmdhd-objs += dhd_linux_sched.o
bcmdhd-objs += dhd_cfg80211.o
bcmdhd-objs += dhd_linux_wq.o
bcmdhd-objs += aiutils.o
bcmdhd-objs += bcmevent.o
bcmdhd-objs += bcmutils.o
bcmdhd-objs += bcmwifi_channels.o
bcmdhd-objs += hndpmu.o
bcmdhd-objs += linux_osl.o
bcmdhd-objs += sbutils.o
bcmdhd-objs += siutils.o
bcmdhd-objs += wl_android.o
bcmdhd-objs += wl_roam.o
bcmdhd-objs += wl_cfg80211.o
bcmdhd-objs += wl_cfgp2p.o
bcmdhd-objs += wl_cfg_btcoex.o
bcmdhd-objs += wldev_common.o
bcmdhd-objs += wl_linux_mon.o
bcmdhd-objs += dhd_linux_platdev.o
bcmdhd-objs += dhd_pno.o
bcmdhd-objs += dhd_rtt.o
bcmdhd-objs += dhd_linux_wq.o
bcmdhd-objs += hnd_pktq.o
bcmdhd-objs += hnd_pktpool.o
bcmdhd-objs += wl_cfgvendor.o
bcmdhd-objs += bcmxtlv.o
	
bcmdhd-$(CONFIG_BCMDHD_SDIO) += bcmsdh.o bcmsdh_linux.o bcmsdh_sdmmc.o bcmsdh_sdmmc_linux.o dhd_cdc.o dhd_wlfc.o dhd_sdio.o
bcmdhd-$(CONFIG_BCMDHD_PCIE) += dhd_pcie.o dhd_pcie_linux.o dhd_msgbuf.o dhd_flowring.o pcie_core.o

obj-$(CONFIG_BCMDHD)  += bcmdhd.o


