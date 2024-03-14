#include "StorageDevice.h"


// if (!mounting)
// {
//     if (isMounted)
//     {
//         // if (AnyFileOpen(&inf.fileSystem))
//         // {
//         //     // Don't re-mount the card if any files are open on it
//         //     reply.copy("SD card has open file(s)");
//         //     return GCodeResult::error;
//         // }
//         // (void)InternalUnmount(card);
//     }

//     mountStartTime = millis();
//     mounting = true;
//     delay(2);
// }