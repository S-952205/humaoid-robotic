import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import NavbarContent from '@theme/Navbar/Content';

// Simple navbar override to ensure proper structure for client-side auth updates
const CustomNavbar = (props) => {
  return (
    <NavbarLayout>
      <NavbarContent {...props}>
        {/* Render the default children (existing navbar items) */}
        {props.children}
      </NavbarContent>
    </NavbarLayout>
  );
};

export default CustomNavbar;