(function() {var implementors = {
"byteorder":[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"byteorder/enum.BigEndian.html\" title=\"enum byteorder::BigEndian\">BigEndian</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"byteorder/enum.LittleEndian.html\" title=\"enum byteorder::LittleEndian\">LittleEndian</a>"]],
"crossbeam_utils":[["impl&lt;T: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"crossbeam_utils/struct.CachePadded.html\" title=\"struct crossbeam_utils::CachePadded\">CachePadded</a>&lt;T&gt;"]],
"heapless":[["impl&lt;const N: <a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"heapless/struct.String.html\" title=\"struct heapless::String\">String</a>&lt;N&gt;"],["impl&lt;P&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"heapless/pool/singleton/arc/struct.Arc.html\" title=\"struct heapless::pool::singleton::arc::Arc\">Arc</a>&lt;P&gt;<span class=\"where fmt-newline\">where\n    P: <a class=\"trait\" href=\"heapless/pool/singleton/arc/trait.Pool.html\" title=\"trait heapless::pool::singleton::arc::Pool\">Pool</a>,\n    P::<a class=\"associatedtype\" href=\"heapless/pool/singleton/arc/trait.Pool.html#associatedtype.Data\" title=\"type heapless::pool::singleton::arc::Pool::Data\">Data</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>,</span>"],["impl&lt;T, const N: <a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"heapless/struct.Vec.html\" title=\"struct heapless::Vec\">Vec</a>&lt;T, N&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>,</span>"],["impl&lt;T&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"heapless/pool/struct.Box.html\" title=\"struct heapless::pool::Box\">Box</a>&lt;T&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>,</span>"],["impl&lt;P&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"heapless/pool/singleton/struct.Box.html\" title=\"struct heapless::pool::singleton::Box\">Box</a>&lt;P&gt;<span class=\"where fmt-newline\">where\n    P: <a class=\"trait\" href=\"heapless/pool/singleton/trait.Pool.html\" title=\"trait heapless::pool::singleton::Pool\">Pool</a>,\n    P::<a class=\"associatedtype\" href=\"heapless/pool/singleton/trait.Pool.html#associatedtype.Data\" title=\"type heapless::pool::singleton::Pool::Data\">Data</a>: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>,</span>"],["impl&lt;T, const N: <a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.usize.html\">usize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"heapless/spsc/struct.Queue.html\" title=\"struct heapless::spsc::Queue\">Queue</a>&lt;T, N&gt;<span class=\"where fmt-newline\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>,</span>"]],
"log":[["impl&lt;'a&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"log/struct.Metadata.html\" title=\"struct log::Metadata\">Metadata</a>&lt;'a&gt;"],["impl&lt;'a&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"log/struct.MetadataBuilder.html\" title=\"struct log::MetadataBuilder\">MetadataBuilder</a>&lt;'a&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"log/enum.Level.html\" title=\"enum log::Level\">Level</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"log/enum.LevelFilter.html\" title=\"enum log::LevelFilter\">LevelFilter</a>"]],
"managed":[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"managed/struct.SlotKey.html\" title=\"struct managed::SlotKey\">Key</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"managed/struct.SlotIndex.html\" title=\"struct managed::SlotIndex\">Slot</a>"]],
"num_complex":[["impl&lt;T: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"num_complex/struct.Complex.html\" title=\"struct num_complex::Complex\">Complex</a>&lt;T&gt;"]],
"num_rational":[["impl&lt;T: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a> + <a class=\"trait\" href=\"num_integer/trait.Integer.html\" title=\"trait num_integer::Integer\">Integer</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"num_rational/struct.Ratio.html\" title=\"struct num_rational::Ratio\">Ratio</a>&lt;T&gt;"]],
"smoltcp":[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/iface/struct.SocketHandle.html\" title=\"struct smoltcp::iface::SocketHandle\">SocketHandle</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.IpEndpoint.html\" title=\"struct smoltcp::wire::IpEndpoint\">Endpoint</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"smoltcp/wire/enum.IpAddress.html\" title=\"enum smoltcp::wire::IpAddress\">Address</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.Ipv4Address.html\" title=\"struct smoltcp::wire::Ipv4Address\">Address</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"smoltcp/wire/enum.IpCidr.html\" title=\"enum smoltcp::wire::IpCidr\">Cidr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.NdiscPrefixInfoFlags.html\" title=\"struct smoltcp::wire::NdiscPrefixInfoFlags\">PrefixInfoFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.Ipv4Cidr.html\" title=\"struct smoltcp::wire::Ipv4Cidr\">Cidr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.NdiscNeighborFlags.html\" title=\"struct smoltcp::wire::NdiscNeighborFlags\">NeighborFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.Ipv6Address.html\" title=\"struct smoltcp::wire::Ipv6Address\">Address</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"smoltcp/wire/enum.IpVersion.html\" title=\"enum smoltcp::wire::IpVersion\">Version</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.IpListenEndpoint.html\" title=\"struct smoltcp::wire::IpListenEndpoint\">ListenEndpoint</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.NdiscRouterFlags.html\" title=\"struct smoltcp::wire::NdiscRouterFlags\">RouterFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.Ipv6Cidr.html\" title=\"struct smoltcp::wire::Ipv6Cidr\">Cidr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"smoltcp/wire/struct.EthernetAddress.html\" title=\"struct smoltcp::wire::EthernetAddress\">Address</a>"]],
"time":[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/struct.PrimitiveDateTime.html\" title=\"struct time::PrimitiveDateTime\">PrimitiveDateTime</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/struct.Duration.html\" title=\"struct time::Duration\">Duration</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/struct.Date.html\" title=\"struct time::Date\">Date</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/error/struct.ComponentRange.html\" title=\"struct time::error::ComponentRange\">ComponentRange</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/struct.UtcOffset.html\" title=\"struct time::UtcOffset\">UtcOffset</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/struct.OffsetDateTime.html\" title=\"struct time::OffsetDateTime\">OffsetDateTime</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"time/struct.Time.html\" title=\"struct time::Time\">Time</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"time/enum.Weekday.html\" title=\"enum time::Weekday\">Weekday</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"time/enum.Month.html\" title=\"enum time::Month\">Month</a>"]],
"x86":[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.PDFlags.html\" title=\"struct x86::bits64::paging::PDFlags\">PDFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/vmx/vmcs/control/struct.SecondaryControls.html\" title=\"struct x86::vmx::vmcs::control::SecondaryControls\">SecondaryControls</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits32/paging/struct.VAddr.html\" title=\"struct x86::bits32::paging::VAddr\">VAddr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/vmx/vmcs/control/struct.ExitControls.html\" title=\"struct x86::vmx::vmcs::control::ExitControls\">ExitControls</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.PML4Flags.html\" title=\"struct x86::bits64::paging::PML4Flags\">PML4Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/segmentation/struct.SegmentSelector.html\" title=\"struct x86::segmentation::SegmentSelector\">SegmentSelector</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.PDPTFlags.html\" title=\"struct x86::bits64::paging::PDPTFlags\">PDPTFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits32/paging/struct.IOAddr.html\" title=\"struct x86::bits32::paging::IOAddr\">IOAddr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits32/paging/struct.PAddr.html\" title=\"struct x86::bits32::paging::PAddr\">PAddr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/vmx/vmcs/control/struct.PrimaryControls.html\" title=\"struct x86::vmx::vmcs::control::PrimaryControls\">PrimaryControls</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/controlregs/struct.Cr4.html\" title=\"struct x86::controlregs::Cr4\">Cr4</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/controlregs/struct.Cr0.html\" title=\"struct x86::controlregs::Cr0\">Cr0</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.PTFlags.html\" title=\"struct x86::bits64::paging::PTFlags\">PTFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/debugregs/struct.Dr6.html\" title=\"struct x86::debugregs::Dr6\">Dr6</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.PML5Flags.html\" title=\"struct x86::bits64::paging::PML5Flags\">PML5Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/rflags/struct.RFlags.html\" title=\"struct x86::bits64::rflags::RFlags\">RFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.PAddr.html\" title=\"struct x86::bits64::paging::PAddr\">PAddr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits32/eflags/struct.EFlags.html\" title=\"struct x86::bits32::eflags::EFlags\">EFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/vmx/vmcs/control/struct.EntryControls.html\" title=\"struct x86::vmx::vmcs::control::EntryControls\">EntryControls</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits32/paging/struct.PDFlags.html\" title=\"struct x86::bits32::paging::PDFlags\">PDFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/vmx/vmcs/control/struct.PinbasedControls.html\" title=\"struct x86::vmx::vmcs::control::PinbasedControls\">PinbasedControls</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/controlregs/struct.Xcr0.html\" title=\"struct x86::controlregs::Xcr0\">Xcr0</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits32/paging/struct.PTFlags.html\" title=\"struct x86::bits32::paging::PTFlags\">PTFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/irq/struct.PageFaultError.html\" title=\"struct x86::irq::PageFaultError\">PageFaultError</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.IOAddr.html\" title=\"struct x86::bits64::paging::IOAddr\">IOAddr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86/bits64/paging/struct.VAddr.html\" title=\"struct x86::bits64::paging::VAddr\">VAddr</a>"]],
"x86_64":[["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/debug/struct.Dr6Flags.html\" title=\"struct x86_64::registers::debug::Dr6Flags\">Dr6Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/idt/struct.PageFaultErrorCode.html\" title=\"struct x86_64::structures::idt::PageFaultErrorCode\">PageFaultErrorCode</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/page_table/struct.PageOffset.html\" title=\"struct x86_64::structures::paging::page_table::PageOffset\">PageOffset</a>"],["impl&lt;S: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> + <a class=\"trait\" href=\"x86_64/structures/paging/page/trait.PageSize.html\" title=\"trait x86_64::structures::paging::page::PageSize\">PageSize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/page/struct.Page.html\" title=\"struct x86_64::structures::paging::page::Page\">Page</a>&lt;S&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/model_specific/struct.CetFlags.html\" title=\"struct x86_64::registers::model_specific::CetFlags\">CetFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/debug/struct.Dr7Flags.html\" title=\"struct x86_64::registers::debug::Dr7Flags\">Dr7Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/model_specific/struct.EferFlags.html\" title=\"struct x86_64::registers::model_specific::EferFlags\">EferFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/addr/struct.PhysAddr.html\" title=\"struct x86_64::addr::PhysAddr\">PhysAddr</a>"],["impl&lt;S: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> + <a class=\"trait\" href=\"x86_64/structures/paging/page/trait.PageSize.html\" title=\"trait x86_64::structures::paging::page::PageSize\">PageSize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/page/struct.PageRange.html\" title=\"struct x86_64::structures::paging::page::PageRange\">PageRange</a>&lt;S&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"x86_64/structures/paging/page_table/enum.PageTableLevel.html\" title=\"enum x86_64::structures::paging::page_table::PageTableLevel\">PageTableLevel</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"x86_64/enum.PrivilegeLevel.html\" title=\"enum x86_64::PrivilegeLevel\">PrivilegeLevel</a>"],["impl&lt;S: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> + <a class=\"trait\" href=\"x86_64/structures/paging/page/trait.PageSize.html\" title=\"trait x86_64::structures::paging::page::PageSize\">PageSize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/frame/struct.PhysFrameRangeInclusive.html\" title=\"struct x86_64::structures::paging::frame::PhysFrameRangeInclusive\">PhysFrameRangeInclusive</a>&lt;S&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/segmentation/struct.SegmentSelector.html\" title=\"struct x86_64::registers::segmentation::SegmentSelector\">SegmentSelector</a>"],["impl&lt;S: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> + <a class=\"trait\" href=\"x86_64/structures/paging/page/trait.PageSize.html\" title=\"trait x86_64::structures::paging::page::PageSize\">PageSize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/frame/struct.PhysFrame.html\" title=\"struct x86_64::structures::paging::frame::PhysFrame\">PhysFrame</a>&lt;S&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/control/struct.Cr0Flags.html\" title=\"struct x86_64::registers::control::Cr0Flags\">Cr0Flags</a>"],["impl&lt;S: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> + <a class=\"trait\" href=\"x86_64/structures/paging/page/trait.PageSize.html\" title=\"trait x86_64::structures::paging::page::PageSize\">PageSize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/page/struct.PageRangeInclusive.html\" title=\"struct x86_64::structures::paging::page::PageRangeInclusive\">PageRangeInclusive</a>&lt;S&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/page_table/struct.PageTableFlags.html\" title=\"struct x86_64::structures::paging::page_table::PageTableFlags\">PageTableFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/rflags/struct.RFlags.html\" title=\"struct x86_64::registers::rflags::RFlags\">RFlags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/xcontrol/struct.XCr0Flags.html\" title=\"struct x86_64::registers::xcontrol::XCr0Flags\">XCr0Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/control/struct.Cr4Flags.html\" title=\"struct x86_64::registers::control::Cr4Flags\">Cr4Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/page_table/struct.PageTableIndex.html\" title=\"struct x86_64::structures::paging::page_table::PageTableIndex\">PageTableIndex</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/gdt/struct.DescriptorFlags.html\" title=\"struct x86_64::structures::gdt::DescriptorFlags\">DescriptorFlags</a>"],["impl&lt;S: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> + <a class=\"trait\" href=\"x86_64/structures/paging/page/trait.PageSize.html\" title=\"trait x86_64::structures::paging::page::PageSize\">PageSize</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/paging/frame/struct.PhysFrameRange.html\" title=\"struct x86_64::structures::paging::frame::PhysFrameRange\">PhysFrameRange</a>&lt;S&gt;"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"enum\" href=\"x86_64/structures/idt/enum.DescriptorTable.html\" title=\"enum x86_64::structures::idt::DescriptorTable\">DescriptorTable</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/structures/idt/struct.SelectorErrorCode.html\" title=\"struct x86_64::structures::idt::SelectorErrorCode\">SelectorErrorCode</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/control/struct.Cr3Flags.html\" title=\"struct x86_64::registers::control::Cr3Flags\">Cr3Flags</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/addr/struct.VirtAddr.html\" title=\"struct x86_64::addr::VirtAddr\">VirtAddr</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/instructions/tlb/struct.Pcid.html\" title=\"struct x86_64::instructions::tlb::Pcid\">Pcid</a>"],["impl <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"x86_64/registers/mxcsr/struct.MxCsr.html\" title=\"struct x86_64::registers::mxcsr::MxCsr\">MxCsr</a>"]],
"zerocopy":[["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.U64.html\" title=\"struct zerocopy::byteorder::U64\">U64</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.I16.html\" title=\"struct zerocopy::byteorder::I16\">I16</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.U32.html\" title=\"struct zerocopy::byteorder::U32\">U32</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.U128.html\" title=\"struct zerocopy::byteorder::U128\">U128</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.I64.html\" title=\"struct zerocopy::byteorder::I64\">I64</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.I128.html\" title=\"struct zerocopy::byteorder::I128\">I128</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.I32.html\" title=\"struct zerocopy::byteorder::I32\">I32</a>&lt;O&gt;"],["impl&lt;O: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a> for <a class=\"struct\" href=\"zerocopy/byteorder/struct.U16.html\" title=\"struct zerocopy::byteorder::U16\">U16</a>&lt;O&gt;"]]
};if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()