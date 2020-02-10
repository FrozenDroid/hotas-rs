use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid_macros::gen_hid_descriptor;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = JOYSTICK) = {
        (collection = PHYSICAL, usage = POINTER) = {
            (usage_page = GENERIC_DESKTOP,) = {
                (usage = X,) = {
                    #[item_settings data,variable,absolute] x=input;
                };
                (usage = Y,) = {
                    #[item_settings data,variable,absolute] y=input;
                };
            }
        }
    }
)]
pub struct JoystickReport {
    pub x: i16,
    pub y: i16,
}
