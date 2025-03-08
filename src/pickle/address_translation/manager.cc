#include "pickle/address_translation/manager.hh"

namespace gem5
{

PickleDeviceAddressTranslationManager::PickleDeviceAddressTranslationManager(
    const PickleDeviceAddressTranslationManagerParams &params
) : SimObject(params),
    is_activated(false)
{
}

PickleDeviceAddressTranslationManager::~PickleDeviceAddressTranslationManager()
{
}

void PickleDeviceAddressTranslationManager::switchOn()
{
    is_activated = true;
}

void PickleDeviceAddressTranslationManager::switchOff()
{
    is_activated = false;
}

}; // namespace gem5
