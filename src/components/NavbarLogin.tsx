import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './NavbarLogin.module.css';

export function NavbarLogin(): JSX.Element {
  const { user, isAuthenticated, signOut } = useAuth();
  const [showDropdown, setShowDropdown] = useState(false);

  if (!isAuthenticated || !user) {
    return (
      <a href="/humanoid-robot-book/auth" className={styles.loginButton}>
        Sign In
      </a>
    );
  }

  return (
    <div className={styles.userMenuContainer}>
      <button
        className={styles.userButton}
        onClick={() => setShowDropdown(!showDropdown)}
        title={user.name}
      >
        {user.image ? (
          <img src={user.image} alt={user.name} className={styles.avatar} />
        ) : (
          <div className={styles.avatarPlaceholder}>
            {user.name.charAt(0).toUpperCase()}
          </div>
        )}
        <span>{user.name.split(' ')[0]}</span>
      </button>

      {showDropdown && (
        <div className={styles.dropdown}>
          <div className={styles.userInfo}>
            {user.image && (
              <img
                src={user.image}
                alt={user.name}
                className={styles.avatarLarge}
              />
            )}
            <div>
              <p className={styles.userName}>{user.name}</p>
              <p className={styles.userEmail}>{user.email}</p>
            </div>
          </div>

          <button
            className={styles.signOutButton}
            onClick={async () => {
              await signOut();
              setShowDropdown(false);
              window.location.href = '/humanoid-robot-book/';
            }}
          >
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
