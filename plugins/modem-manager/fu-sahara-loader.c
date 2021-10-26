/*
 * Copyright (C) 2021 Quectel Wireless Solutions Co., Ltd.
 *                    Ivan Mikhanchuk <ivan.mikhanchuk@quectel.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#include "config.h"

#include <string.h>
#include <sys/ioctl.h>
#include <linux/usb/ch9.h>
#include <linux/usbdevice_fs.h>

#include "fu-sahara-loader.h"

#define SAHARA_VERSION 2
#define SAHARA_VERSION_COMPATIBLE 1

#define SAHARA_RAW_BUFFER_SIZE (4*1024)

#define IO_TIMEOUT_MS 15000

struct _FuSaharaLoader {
	GObject parent_instance;

	gchar *sysfs_path;

	int ifnum;
	int in_ep;
	int out_ep;

	gsize in_maxpktsize;
	gsize out_maxpktsize;

	FuIOChannel *io_channel;
};

G_DEFINE_TYPE(FuSaharaLoader, fu_sahara_loader, G_TYPE_OBJECT)

/* Protocol definitions */
typedef enum {
	SAHARA_NO_CMD_ID = 0,
	SAHARA_HELLO_ID,
	SAHARA_HELLO_RESPONSE_ID,
	SAHARA_READ_DATA_ID,
	SAHARA_END_OF_IMAGE_TX_ID,
	SAHARA_DONE_ID,
	SAHARA_DONE_RESP_ID,
	SAHARA_RESET_ID,
	SAHARA_RESET_RESPONSE_ID,
	SAHARA_READ_DATA_64_BIT_ID = 0x12,
	SAHARA_LAST_CMD_ID
} FuSaharaCommandId;

typedef enum {
	SAHARA_STATUS_SUCCESS = 0,
	SAHARA_STATUS_FAILED,
	SAHARA_STATUS_LAST
} FuSaharaStatusCode;

typedef enum {
	SAHARA_MODE_IMAGE_TX_PENDING,
	SAHARA_MODE_IMAGE_TX_COMPLETE,
	SAHARA_MODE_LAST
} FuSaharaMode;

/* Sahara packet definition */
struct sahara_packet {
	guint32 command_id;
	guint32 length;

	union {
		struct { 
			guint32 version;
			guint32 version_compatible;
			guint32 max_packet_length;
			guint32 mode;
		} hello;
		struct {
			guint32 version;
			guint32 version_compatible;
			guint32 status;
			guint32 mode;
		} hello_response;
		struct {
			guint32 image_id;
			guint32 offset;
			guint32 length;
		} read_data;
		struct {
			guint32 image_id;
			guint32 status;
		} end_of_image_transfer;
		/* done packet = header only */
		struct {
			guint32 image_transfer_status;
		} done_response;
		/* reset packet = header only */
		/* reset response packet = header only */
		struct {
			guint64 image_id;
			guint64 offset;
			guint64 length;
		} read_data_64bit;
	};
} __attribute((packed));

/* helper functions */
static const gchar *command_id_to_str[SAHARA_LAST_CMD_ID] = {
	"SAHARA_NO_CMD_ID",
	"SAHARA_HELLO_ID",
	"SAHARA_HELLO_RESP_ID",
	"SAHARA_READ_DATA_ID",
	"SAHARA_END_IMAGE_TX_ID",
	"SAHARA_DONE_ID",
	"SAHARA_DONE_RESP_ID",
	"SAHARA_RESET_ID",
	"SAHARA_RESET_RESP_ID"
};

static const gchar *mode_to_str[SAHARA_MODE_LAST] = {
	"IMAGE_TX_PENDING",
	"IMAGE_TX_COMPLETE"
};

static void
fu_sahara_loader_log_packet(struct sahara_packet *packet, const gchar *prefix)
{
	if (packet == NULL)
		g_debug("%s (null)", prefix);
	else if (packet->command_id < SAHARA_LAST_CMD_ID) { 
		const gchar *cmd_name =  command_id_to_str[packet->command_id];

		if (packet->command_id == SAHARA_HELLO_ID && packet->hello.mode < SAHARA_MODE_LAST)
			g_debug("%s %s mode: %s", prefix, cmd_name, mode_to_str[packet->hello.mode]);
		else if (packet->command_id == SAHARA_HELLO_RESPONSE_ID &&
				packet->hello_response.mode < SAHARA_MODE_LAST)
			g_debug("%s %s mode: %s", prefix, cmd_name, mode_to_str[packet->hello_response.mode]);
		else if (packet->command_id == SAHARA_END_OF_IMAGE_TX_ID)
			g_debug("%s %s image_id = %u, status = %s (%u)", prefix, cmd_name,
				packet->end_of_image_transfer.image_id,
				packet->end_of_image_transfer.status == 0 ? "SUCCESS" : "FAILED",
				packet->end_of_image_transfer.status);
		else
			g_debug("%s %s", prefix, cmd_name);
	} else
		g_debug("%s unknown command", prefix);
}

static FuSaharaCommandId
sahara_packet_get_command_id(GByteArray *packet)
{
	return ((struct sahara_packet *)(packet->data))->command_id;
}

static FuSaharaCommandId
sahara_packet_get_length(GByteArray *packet)
{
	return ((struct sahara_packet *)(packet->data))->length;
}

/* initialization */
static gboolean
fu_sahara_loader_parse_usb_desc(FuSaharaLoader *self, GByteArray *usb_dev_desc, GError **error)
{
	struct usb_descriptor_header *hdr;
	struct usb_device_descriptor *dev;
	struct usb_interface_descriptor *ifc;
	struct usb_endpoint_descriptor *ept;

	guint8 ifclass = 0;
	guint8 ifsubclass = 0;
	guint8 ifnum = 0;

	guint8 *ptr = usb_dev_desc->data;
	guint8 *end = usb_dev_desc->data + usb_dev_desc->len;

	while (ptr < end) {
		hdr = (void *)ptr;

		if (hdr->bDescriptorType == USB_DT_DEVICE) {
			dev = (void *)ptr;

			if (dev->idVendor != 0x05c6 || dev->idProduct != 0x9008) {
				g_set_error(error,
					G_IO_ERROR,
					G_IO_ERROR_FAILED,
					"Wrong device and/or vendor id: 0x%04x 0x%04x",
					dev->idVendor, dev->idProduct);
				return FALSE;
			}
		} else if (hdr->bDescriptorType == USB_DT_INTERFACE && hdr->bLength == USB_DT_INTERFACE_SIZE) {
			ifc = (void *)ptr;

			ifclass = ifc->bInterfaceClass;
			ifsubclass = ifc->bInterfaceSubClass;
			ifnum = ifc->bInterfaceNumber;

			self->in_ep = -1;
			self->out_ep = -1;
		} else if (hdr->bDescriptorType == USB_DT_ENDPOINT && hdr->bLength == USB_DT_ENDPOINT_SIZE ) {
			ept = (void *)ptr;

			if (ifclass == 0xff && ifsubclass == 0xff) {
				if ((ept->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) {
					if (ept->bEndpointAddress & USB_ENDPOINT_DIR_MASK) {
						self->in_ep = ept->bEndpointAddress;
						self->in_maxpktsize = ept->wMaxPacketSize;
					} else {
						self->out_ep = ept->bEndpointAddress;
						self->out_maxpktsize = ept->wMaxPacketSize;
					}

					if (self->in_ep != -1 && self->out_ep != -1) {
						self->ifnum = ifnum;
						return TRUE;
					}
				}
			}
		}

		ptr += hdr->bLength;
	}

	g_set_error(error,
		G_IO_ERROR,
		G_IO_ERROR_FAILED,
		"Couldn't find USB interface and endpoint for Sahara protocol");

	return FALSE;
}

static gboolean
fu_sahara_loader_claim_interface(FuSaharaLoader *self, GError **error)
{
	int res;
	struct usbdevfs_ioctl cmd;

	cmd.ifno = self->ifnum;
	cmd.ioctl_code = USBDEVFS_DISCONNECT;
	cmd.data = NULL;

	res = ioctl(fu_io_channel_unix_get_fd(self->io_channel), USBDEVFS_IOCTL, &cmd);
	if (res && errno != ENODATA) {
		g_set_error(error,
			G_IO_ERROR,
			G_IO_ERROR_FAILED,
			"Failed to disconnect kernel driver");
		return FALSE;
	}

	res = ioctl(fu_io_channel_unix_get_fd(self->io_channel), USBDEVFS_CLAIMINTERFACE, &self->ifnum);
	if (res < 0) {
		g_set_error(error,
			G_IO_ERROR,
			G_IO_ERROR_FAILED,
			"Failed to claim USB interface");
		return FALSE;
	}

	return TRUE;
}

gboolean
fu_sahara_loader_open(FuSaharaLoader *self, GError **error)
{
	const gchar *devname;
	g_autoptr(GUdevClient) client = NULL;
	g_autoptr(GUdevDevice) device = NULL;
	g_autoptr(GByteArray) usb_desc = NULL;

	g_debug("opening sahara port...");

	client = g_udev_client_new(NULL);
	device = g_udev_client_query_by_sysfs_path(client, self->sysfs_path);
	if (device == NULL) {
		g_set_error(error,
			    G_IO_ERROR,
			    G_IO_ERROR_FAILED,
			    "Failed to get device from the path: %s",
			    self->sysfs_path);
		return FALSE;
	}

	devname = g_udev_device_get_property(device, "DEVNAME");
	if (devname == NULL) {
		g_set_error(error,
			    G_IO_ERROR,
			    G_IO_ERROR_FAILED,
			    "Failed to get the devname for the device: %s",
			    self->sysfs_path);
		return FALSE;
	}

	g_debug("Sahara device devname = %s", devname);

	self->io_channel = fu_io_channel_new_file(devname, error);
	if (self->io_channel == NULL) {
		g_prefix_error(error, "Failed to open %s: ", devname);
		return FALSE;
	}
	usb_desc = fu_io_channel_read_byte_array(self->io_channel, 1024, 1500, FU_IO_CHANNEL_FLAG_USE_BLOCKING_IO, error);
	if (usb_desc == NULL) {
		g_prefix_error(error, "Failed to read from the Sahara device: ");
		return FALSE;
	}

	if (!fu_sahara_loader_parse_usb_desc(self, usb_desc, error)) {
		g_prefix_error(error, "Failed to parse USB descriptor: ");
		return FALSE;
	}

	if (!fu_sahara_loader_claim_interface(self, error))
		return FALSE;

	return TRUE;
}

gboolean
fu_sahara_loader_close(FuSaharaLoader *self, GError **error)
{
	g_debug("closing sahara port...");
	if (!fu_io_channel_shutdown(self->io_channel, error))
		return FALSE;
	g_clear_object(&self->io_channel);
	return TRUE;
}

/* IO functions */
gboolean
fu_sahara_loader_qdl_is_open(FuSaharaLoader *self)
{
	g_return_val_if_fail(self != NULL, FALSE);

	return self->io_channel != NULL;
}

GByteArray *
fu_sahara_loader_qdl_read(FuSaharaLoader *self, GError **error)
{
	int res;
	struct usbdevfs_bulktransfer bulk = {};
	g_autoptr(GByteArray) buf = g_byte_array_sized_new(SAHARA_RAW_BUFFER_SIZE);
	g_byte_array_set_size(buf, SAHARA_RAW_BUFFER_SIZE);

	bulk.ep = self->in_ep;
	bulk.len = buf->len;
	bulk.data = buf->data;
	bulk.timeout = IO_TIMEOUT_MS;

	res = ioctl(fu_io_channel_unix_get_fd(self->io_channel), USBDEVFS_BULK, &bulk);
	if (res < 0) {
		g_set_error(error,
			    G_IO_ERROR,
			    G_IO_ERROR_FAILED,
			    "Failed to receive packet: %s",
			    strerror(errno));
		return NULL;
	}
	g_byte_array_set_size(buf, res);

	if (g_getenv("FWUPD_MODEM_MANAGER_VERBOSE") != NULL)
		g_debug("read %i bytes", res);

	return g_steal_pointer(&buf);
}

gboolean
fu_sahara_loader_qdl_write(FuSaharaLoader *self, const guint8 *data, gsize sz, GError **error)
{
	struct usbdevfs_bulktransfer bulk = {};
	gsize total_sent = 0;

	while(total_sent < sz) {
		int res;
		gsize rem = sz - total_sent;
		int transfer_size = (rem > self->out_maxpktsize) ? self->out_maxpktsize : rem;

		bulk.ep = self->out_ep;
		bulk.len = transfer_size;
		bulk.data = (void*)(&data[total_sent]);
		bulk.timeout = IO_TIMEOUT_MS;

		res = ioctl(fu_io_channel_unix_get_fd(self->io_channel), USBDEVFS_BULK, &bulk);
		if(res != transfer_size) {
			g_set_error(error,
				    G_IO_ERROR,
				    G_IO_ERROR_FAILED,
				    "Write error : retval = %i, error: %s",
				    res, strerror(errno));
			return FALSE;
		}
		total_sent += transfer_size;

		if (g_getenv("FWUPD_MODEM_MANAGER_VERBOSE") != NULL)
			g_debug("sent %04i bytes, total = %04lu", res, total_sent);
	}

	if (sz % self->out_maxpktsize == 0) {
		bulk.ep = self->out_ep;
		bulk.len = 0;
		bulk.data = NULL;
		bulk.timeout = IO_TIMEOUT_MS;

		ioctl(fu_io_channel_unix_get_fd(self->io_channel), USBDEVFS_BULK, &bulk);
		if (g_getenv("FWUPD_MODEM_MANAGER_VERBOSE") != NULL)
			g_debug("sent zlp");
	}

	return TRUE;
}

static gboolean
fu_sahara_loader_write_prog(FuSaharaLoader *self, guint32 offset, guint32 length, GBytes *prog, GError **error)
{
	gsize sz;
	const guint8 *data = g_bytes_get_data(prog, &sz);

	g_return_val_if_fail(offset + length <= sz, FALSE);

	g_debug("SENDING --> RAW_DATA: %u bytes (%u/%lu)", length, offset, sz);
	return fu_sahara_loader_qdl_write(self, &data[offset], length, error);
}

static gboolean
fu_sahara_loader_send_packet(FuSaharaLoader *self, GByteArray *pkt, GError **error)
{
	const guint8 *data = pkt->data;
	gsize sz = pkt->len;

	fu_sahara_loader_log_packet((struct sahara_packet *)data, "SENDING -->");
	return fu_sahara_loader_qdl_write(self, data, sz, error);
}

/* packet composers */
static GByteArray *
fu_sahara_loader_compose_reset_packet(void)
{
	GByteArray *self;
	guint32 len = 0x08;

	self = g_byte_array_sized_new(len);
	g_byte_array_set_size (self, len);

	((struct sahara_packet *)(self->data))->command_id = GUINT32_TO_LE (SAHARA_RESET_ID);
	((struct sahara_packet *)(self->data))->length = GUINT32_TO_LE (len);

	return self;
}

static GByteArray *
fu_sahara_loader_compose_hello_response_packet(FuSaharaMode mode)
{
	GByteArray *self;
	guint32 len = 0x30;

	self = g_byte_array_sized_new(len);
	g_byte_array_set_size (self, len);

	((struct sahara_packet *)(self->data))->command_id = GUINT32_TO_LE (SAHARA_HELLO_RESPONSE_ID);
	((struct sahara_packet *)(self->data))->length = GUINT32_TO_LE (len);

	((struct sahara_packet *)(self->data))->hello_response.version = GUINT32_TO_LE(SAHARA_VERSION);
	((struct sahara_packet *)(self->data))->hello_response.version_compatible = GUINT32_TO_LE(SAHARA_VERSION_COMPATIBLE);
	((struct sahara_packet *)(self->data))->hello_response.status = GUINT32_TO_LE(SAHARA_STATUS_SUCCESS);
	((struct sahara_packet *)(self->data))->hello_response.mode = GUINT32_TO_LE(SAHARA_MODE_IMAGE_TX_PENDING);

	return self;
}

static GByteArray *
fu_sahara_loader_compose_done_packet(void)
{
	GByteArray *self;
	guint32 len = 0x08;

	self = g_byte_array_sized_new(len);
	g_byte_array_set_size (self, len);

	((struct sahara_packet *)(self->data))->command_id = GUINT32_TO_LE (SAHARA_DONE_ID);
	((struct sahara_packet *)(self->data))->length = GUINT32_TO_LE (len);

	return self;
}

static gboolean
fu_sahara_loader_send_reset_packet(FuSaharaLoader *self, GError **error)
{
	g_autoptr(GByteArray) rx_packet = NULL;
	g_autoptr(GByteArray) tx_packet = NULL;

	tx_packet = fu_sahara_loader_compose_reset_packet();
	if (!fu_sahara_loader_send_packet(self, tx_packet, error)) {
		g_prefix_error(error, "Failed to send reset packet: ");
		return FALSE;
	}

	rx_packet = fu_sahara_loader_qdl_read(self, error);
	if (rx_packet == NULL || sahara_packet_get_command_id(rx_packet) != SAHARA_RESET_RESPONSE_ID) {
		g_set_error(error,
			    G_IO_ERROR,
			    G_IO_ERROR_FAILED,
			    "Failed to receive RESET_RESPONSE packet");
		return FALSE;
	}

	g_debug("Reset succeeded");
	return TRUE;
}

static gboolean
fu_sahara_loader_wait_hello_rsp(FuSaharaLoader *self, GError **error)
{
	g_autoptr(GByteArray) rx_packet = NULL;
	g_autoptr(GByteArray) tx_packet = NULL;

	rx_packet = fu_sahara_loader_qdl_read(self, error); 
	if (rx_packet == NULL) {
		g_autoptr(GByteArray) ping = NULL;
		ping = g_byte_array_sized_new(1);
		g_byte_array_set_size (ping, 1);
		fu_sahara_loader_send_packet(self, ping, NULL);
		rx_packet = fu_sahara_loader_qdl_read(self, error); 
	}

	g_return_val_if_fail (rx_packet != NULL, FALSE);

	fu_sahara_loader_log_packet((struct sahara_packet *)(rx_packet->data), "RECEIVED <--");

	if (sahara_packet_get_command_id(rx_packet) != SAHARA_HELLO_ID) {
		g_set_error(error,
			    G_IO_ERROR,
			    G_IO_ERROR_FAILED,
			    "Received a different packet while waiting for the HELLO packet");
		fu_sahara_loader_send_reset_packet(self, NULL);
		return FALSE;
	}

	tx_packet = fu_sahara_loader_compose_hello_response_packet(SAHARA_MODE_IMAGE_TX_PENDING);

	return fu_sahara_loader_send_packet(self, tx_packet, error);
}

/* main routine */
gboolean
fu_sahara_loader_run(FuSaharaLoader *self, GBytes *prog, GError **error)
{
	gboolean done = FALSE;
	g_autoptr(GError) err_local = NULL;

	g_return_val_if_fail (prog != NULL, FALSE);

	g_debug("STATE -- SAHARA_WAIT_HELLO");
	if (!fu_sahara_loader_wait_hello_rsp(self, error))
		return FALSE;

	while (!done) {
		guint32 command_id;
		g_autoptr(GByteArray) rx_packet = NULL;
		g_autoptr(GByteArray) tx_packet = NULL;

		g_debug("STATE -- SAHARA_WAIT_COMMAND");
		rx_packet = fu_sahara_loader_qdl_read(self, error);
		if (rx_packet == NULL)
			break;
		if (rx_packet->len != sahara_packet_get_length(rx_packet)) {
			g_set_error(error,
				    G_IO_ERROR,
				    G_IO_ERROR_FAILED,
				    "Received packet length is not matching");
			break;
		}

		fu_sahara_loader_log_packet((struct sahara_packet *)(rx_packet->data), "RECEIVED <--");

		command_id = sahara_packet_get_command_id(rx_packet);
		if (command_id == SAHARA_HELLO_ID) {
			tx_packet = fu_sahara_loader_compose_hello_response_packet(SAHARA_MODE_IMAGE_TX_PENDING);
			fu_sahara_loader_send_packet(self, tx_packet, &err_local);
		} else if (command_id == SAHARA_READ_DATA_ID) {
			guint32 offset = ((struct sahara_packet *)(rx_packet->data))->read_data.offset;
			guint32 length = ((struct sahara_packet *)(rx_packet->data))->read_data.length;
			fu_sahara_loader_write_prog(self, offset, length, prog, &err_local);
		} else if (command_id == SAHARA_READ_DATA_64_BIT_ID) {
			guint64 offset = ((struct sahara_packet *)(rx_packet->data))->read_data_64bit.offset;
			guint64 length = ((struct sahara_packet *)(rx_packet->data))->read_data_64bit.length;
			fu_sahara_loader_write_prog(self, offset, length, prog, &err_local);
		} else if (command_id == SAHARA_END_OF_IMAGE_TX_ID) {
			guint32 status = ((struct sahara_packet *)(rx_packet->data))->end_of_image_transfer.status;
			if (status == SAHARA_STATUS_SUCCESS) {
				tx_packet = fu_sahara_loader_compose_done_packet();
				fu_sahara_loader_send_packet(self, tx_packet, &err_local);
			}
		} else if (command_id == SAHARA_DONE_RESP_ID) {
			done = TRUE;
		} else {
			g_warning("Unexpected packet received: cmd_id = %u, len = %u",
				  command_id,
				  sahara_packet_get_length(rx_packet));
		}

		if (err_local != NULL) {
			g_warning("%s", err_local->message);
			g_clear_error(&err_local);
		}
	}

	if (!done)
		fu_sahara_loader_send_reset_packet(self, NULL);

	return done;
}

static void
fu_sahara_loader_init(FuSaharaLoader *self)
{
}

static void
fu_sahara_loader_finalize(GObject *object)
{
	FuSaharaLoader *self = FU_SAHARA_LOADER(object);
	g_warn_if_fail(self->io_channel == NULL);
	g_free(self->sysfs_path);
	G_OBJECT_CLASS(fu_sahara_loader_parent_class)->finalize(object);
}

static void
fu_sahara_loader_class_init(FuSaharaLoaderClass *klass)
{
	GObjectClass *object_class = G_OBJECT_CLASS(klass);
	object_class->finalize = fu_sahara_loader_finalize;
}

FuSaharaLoader *
fu_sahara_loader_new(const gchar *device_sysfs_path)
{
	FuSaharaLoader *self = g_object_new(FU_TYPE_SAHARA_LOADER, NULL);
	self->sysfs_path = g_strdup(device_sysfs_path);
	return self;
}
