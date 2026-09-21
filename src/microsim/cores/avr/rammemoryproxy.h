#ifndef RAMMEMORYPROXY_H
#define RAMMEMORYPROXY_H

#include <cstdint>

class RamMemoryProxy;

// 1. Fully define RamMemoryProxy first so its methods are visible
class RamMemoryProxy
{
public:
    virtual ~RamMemoryProxy() = default;

    virtual uint8_t read(uint16_t address) = 0;
    virtual void write(uint16_t address, uint8_t value) = 0;
    virtual void reset() = 0;

    inline class RamProxyRef operator[](uint16_t address);
};

// 2. Define RamProxyRef helper implementation after RamMemoryProxy is known
class RamProxyRef
{
public:
    inline RamProxyRef(RamMemoryProxy& proxy, uint16_t address)
        : m_proxy(proxy), m_address(address) {}

    inline RamProxyRef& operator=(uint8_t value)
    {
        m_proxy.write(m_address, value);
        return *this;
    }

    inline operator uint8_t() const
    {
        return m_proxy.read(m_address);
    }

    inline RamProxyRef& operator|=(uint8_t value)
    {
        uint8_t current = m_proxy.read(m_address);
        m_proxy.write(m_address, current | value);
        return *this;
    }

    inline RamProxyRef& operator&=(uint8_t value)
    {
        uint8_t current = m_proxy.read(m_address);
        m_proxy.write(m_address, current & value);
        return *this;
    }

    inline RamProxyRef& operator+=(uint8_t value)
    {
        uint8_t current = m_proxy.read(m_address);
        m_proxy.write(m_address, current + value);
        return *this;
    }

    inline RamProxyRef& operator-=(uint8_t value)
    {
        uint8_t current = m_proxy.read(m_address);
        m_proxy.write(m_address, current - value);
        return *this;
    }

private:
    RamMemoryProxy& m_proxy;
    uint16_t m_address;
};

// 3. Inline operator[] implementation for RamMemoryProxy
inline RamProxyRef RamMemoryProxy::operator[](uint16_t address)
{
    return RamProxyRef(*this, address);
}

class DirectRamProxy : public RamMemoryProxy
{
public:
    DirectRamProxy(uint8_t* rawRam = nullptr, uint32_t ramSize = 0)
        : m_ram(rawRam), m_size(ramSize) {}

    void setBuffer(uint8_t* rawRam, uint32_t ramSize)
    {
        m_ram = rawRam;
        m_size = ramSize;
    }

    inline uint8_t read(uint16_t address) override
    {
        if (m_ram && address < m_size) {
            return m_ram[address];
        }
        return 0xFF;
    }

    inline void write(uint16_t address, uint8_t value) override
    {
        if (m_ram && address < m_size) {
            m_ram[address] = value;
        }
    }

    void reset() override {}

private:
    uint8_t* m_ram;
    uint32_t m_size;
};

#endif // RAMMEMORYPROXY_H

