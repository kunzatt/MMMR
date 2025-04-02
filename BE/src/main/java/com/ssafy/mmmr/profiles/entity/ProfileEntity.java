package com.ssafy.mmmr.profiles.entity;

import java.time.LocalDateTime;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.ssafy.mmmr.account.entity.AccountEntity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.EnumType;
import jakarta.persistence.Enumerated;
import jakarta.persistence.FetchType;
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
@Table(name = "profiles")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
public class ProfileEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private Long id;

	@ManyToOne(fetch = FetchType.LAZY)
	@JoinColumn(name = "account_id", nullable = false)
	@JsonIgnore
	private AccountEntity account;

	@Column(name = "nickname", nullable = false, length = 20)
	private String nickname;

	@Enumerated(EnumType.STRING)
	@Column(name = "callsign", nullable = false)
	private CallSign callSign;

	@Column(name = "deleted", nullable = false, columnDefinition = "BOOLEAN DEFAULT FALSE")
	private Boolean deleted = false;

	@Column(name = "count", nullable = false)
	private Integer count;

	@CreatedDate
	@Column(name = "created_at", nullable = false, updatable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(name = "updated_at")
	private LocalDateTime updatedAt;

	@Builder
	public ProfileEntity(AccountEntity account, String nickname, CallSign callSign) {
		this.account = account;
		this.nickname = nickname;
		this.callSign = callSign;
		this.count = 0;
	}

	public void delete() {
		this.deleted = true;
	}

	public void changeNickname(String nickname) {
		this.nickname = nickname;
	}

	public void changeCallSign(CallSign callSign) {
		this.callSign = callSign;
	}

	public void decreaseTransportationCount() {
		if (this.count > 0) {
			this.count--;
		}
	}

}