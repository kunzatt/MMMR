package com.ssafy.mmmr.devices.entity;

import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.profiles.entity.ProfileEntity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Table(name = "home_devices")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class HomeDeviceEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private Long id;

	@ManyToOne
	@JoinColumn(name = "account_id", nullable = false)
	private AccountEntity account;

	@Column(name = "device", nullable = false)
	private String device;

	@Column(name = "turned", nullable = false)
	private String turned;

	@Builder
	public HomeDeviceEntity(AccountEntity account, String device, String turned) {
		this.account = account;
		this.device = device;
		this.turned = turned;
	}

	public void update(String turned) {
		this.turned = turned;
	}
}